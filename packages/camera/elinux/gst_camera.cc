// Copyright 2022 Sony Group Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "gst_camera.h"

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <iostream>

GstCamera::GstCamera(std::unique_ptr<CameraStreamHandler> handler)
    : stream_handler_(std::move(handler)) {
  gst_.pipeline = nullptr;
  gst_.camerabin = nullptr;
  gst_.video_convert = nullptr;
  gst_.video_sink = nullptr;
  gst_.video_valve = nullptr;
  gst_.output = nullptr;
  gst_.bus = nullptr;
  gst_.buffer = nullptr;

  if (!CreatePipeline()) {
    std::cerr << "Failed to create a pipeline" << std::endl;
    DestroyPipeline();
    return;
  }

  // Prerolls before getting information from the pipeline.
  Preroll();

  GetZoomMaxMinSize(max_zoom_level_, min_zoom_level_);
}

GstCamera::~GstCamera() {
  // If we are still recording, stop cleanly before tearing down.
  if (is_recording_) {
    // Re-open the valve so the EOS event can flow through to the muxer.
    if (gst_.video_valve) {
      g_object_set(gst_.video_valve, "drop", FALSE, NULL);
    }
    g_signal_emit_by_name(gst_.camerabin, "stop-capture", NULL);
    WaitForVideoDone(2);
    g_object_set(gst_.camerabin, "mode", 1, NULL);
    is_recording_ = false;
    is_recording_paused_ = false;
  }
  Stop();
  DestroyPipeline();
}

// static
void GstCamera::GstLibraryLoad() { gst_init(NULL, NULL); }

// static
void GstCamera::GstLibraryUnload() { gst_deinit(); }

bool GstCamera::Play() {
  auto result = gst_element_set_state(gst_.pipeline, GST_STATE_PLAYING);
  if (result == GST_STATE_CHANGE_FAILURE) {
    std::cerr << "Failed to change the state to PLAYING" << std::endl;
    return false;
  }

  // Waits until the state becomes GST_STATE_PLAYING.
  if (result == GST_STATE_CHANGE_ASYNC) {
    GstState state;
    result =
        gst_element_get_state(gst_.pipeline, &state, NULL, GST_CLOCK_TIME_NONE);
    if (result == GST_STATE_CHANGE_FAILURE) {
      std::cerr << "Failed to get the current state" << std::endl;
    }
  }

  return true;
}

bool GstCamera::Pause() {
  if (gst_element_set_state(gst_.pipeline, GST_STATE_PAUSED) ==
      GST_STATE_CHANGE_FAILURE) {
    std::cerr << "Failed to change the state to PAUSED" << std::endl;
    return false;
  }
  return true;
}

bool GstCamera::Stop() {
  if (gst_element_set_state(gst_.pipeline, GST_STATE_READY) ==
      GST_STATE_CHANGE_FAILURE) {
    std::cerr << "Failed to change the state to READY" << std::endl;
    return false;
  }
  return true;
}

void GstCamera::TakePicture(OnNotifyCaptured on_notify_captured) {
  if (!gst_.camerabin) {
    std::cerr << "Failed to take a picture" << std::endl;
    return;
  }

  on_notify_captured_ = on_notify_captured;
  gchar* raw = g_strdup_printf("captured_%04u.jpg", captured_count_++);
  std::string filename(raw);
  g_free(raw);
  g_object_set(gst_.camerabin, "location", filename.c_str(), NULL);
  g_signal_emit_by_name(gst_.camerabin, "start-capture", NULL);
}

// ---------------------------------------------------------------------------
//  Video recording: start / stop / pause / resume
//
//  Camerabin has two modes:
//    mode=1  ->  image capture (default)
//    mode=2  ->  video capture
//
//  Signals:
//    "start-capture"  begins capture in the current mode
//    "stop-capture"   ends an ongoing video capture (ignored in image mode)
//
//  Bus messages:
//    "image-done"  emitted after an image has been written
//    "video-done"  emitted after the video muxer has flushed and closed
//
//  Pause / resume strategy:
//    Camerabin does NOT support native pause/resume.  Calling
//    stop-capture finalizes the file, and start-capture overwrites it.
//
//    Instead we use a GStreamer "valve" element set as camerabin's
//    "video-filter" property.  This element sits in the recording path
//    only (NOT the viewfinder path), so toggling its "drop" property
//    drops buffers going to the encoder while the preview keeps running.
//
//    Pause:  valve drop=TRUE  → frames are silently discarded.
//    Resume: valve drop=FALSE → frames flow to the encoder again.
//    Stop:   valve drop=FALSE first (so EOS can flow through to the
//            muxer for proper finalization), then stop-capture.
//
//    The resulting video will have a time-jump at the pause point
//    but will NOT contain frames from the paused period.
// ---------------------------------------------------------------------------

bool GstCamera::StartVideoRecording(const std::string& file_path) {
  if (!gst_.camerabin) {
    std::cerr << "Failed to start video recording: no camerabin" << std::endl;
    return false;
  }
  if (is_recording_) {
    std::cerr << "Already recording" << std::endl;
    return false;
  }

  video_file_path_ = file_path;

  // Make sure the recording-path valve is open.
  if (gst_.video_valve) {
    g_object_set(gst_.video_valve, "drop", FALSE, NULL);
  }

  // Switch camerabin to video mode.
  g_object_set(gst_.camerabin, "mode", 2, NULL);
  g_object_set(gst_.camerabin, "location", video_file_path_.c_str(), NULL);

  // Reset the done-flag before starting.
  {
    std::lock_guard<std::mutex> lock(video_done_mutex_);
    video_done_received_ = false;
  }

  g_signal_emit_by_name(gst_.camerabin, "start-capture", NULL);

  is_recording_ = true;
  is_recording_paused_ = false;
  std::cout << "Video recording started: " << video_file_path_ << std::endl;
  return true;
}

void GstCamera::StopVideoRecording(OnNotifyCaptured on_video_done) {
  if (!gst_.camerabin) {
    std::cerr << "Failed to stop video recording: no camerabin" << std::endl;
    return;
  }
  if (!is_recording_) {
    std::cerr << "Not recording" << std::endl;
    if (on_video_done) {
      on_video_done("");
    }
    return;
  }

  // If we are paused (valve dropping buffers), re-open the valve so
  // the EOS event from stop-capture can flow through to the muxer.
  // Without this the muxer never receives EOS and the file is truncated.
  if (is_recording_paused_ && gst_.video_valve) {
    g_object_set(gst_.video_valve, "drop", FALSE, NULL);
  }

  // Reset the done-flag before requesting the stop.
  {
    std::lock_guard<std::mutex> lock(video_done_mutex_);
    video_done_received_ = false;
  }

  // Tell camerabin to stop capturing.  This starts an async drain of the
  // encoder and muxer — all in-flight buffers will be written out.
  g_signal_emit_by_name(gst_.camerabin, "stop-capture", NULL);
  is_recording_ = false;
  is_recording_paused_ = false;
  std::cout << "Video recording stop requested, waiting for muxer flush..."
            << std::endl;

  // Wait for the "video-done" bus message (up to 5 seconds).  During this
  // time the pipeline stays PLAYING so the viewfinder keeps working and
  // the encoder/muxer can finish writing the remaining frames.
  WaitForVideoDone(5);

  // Now it is safe to switch back to image mode.
  g_object_set(gst_.camerabin, "mode", 1, NULL);
  std::cout << "Video recording stopped, file: " << video_file_path_
            << std::endl;

  // Deliver the result to the caller.
  if (on_video_done) {
    on_video_done(video_file_path_);
  }
}

bool GstCamera::PauseVideoRecording() {
  if (!gst_.camerabin || !is_recording_) {
    std::cerr << "Cannot pause: not recording" << std::endl;
    return false;
  }
  if (is_recording_paused_) {
    std::cerr << "Already paused" << std::endl;
    return true;
  }

  // Drop all buffers in the recording path via the valve element.
  // The viewfinder is unaffected because the valve sits in the
  // video-filter path only.
  if (gst_.video_valve) {
    g_object_set(gst_.video_valve, "drop", TRUE, NULL);
  }

  is_recording_paused_ = true;
  std::cout << "Video recording paused (valve drop=TRUE)" << std::endl;
  return true;
}

bool GstCamera::ResumeVideoRecording() {
  if (!gst_.camerabin || !is_recording_) {
    std::cerr << "Cannot resume: not recording" << std::endl;
    return false;
  }
  if (!is_recording_paused_) {
    std::cerr << "Not paused" << std::endl;
    return true;
  }

  // Re-open the valve so buffers flow to the encoder again.
  if (gst_.video_valve) {
    g_object_set(gst_.video_valve, "drop", FALSE, NULL);
  }

  is_recording_paused_ = false;
  std::cout << "Video recording resumed (valve drop=FALSE)" << std::endl;
  return true;
}

bool GstCamera::WaitForVideoDone(int timeout_seconds) {
  std::unique_lock<std::mutex> lock(video_done_mutex_);
  bool done = video_done_cv_.wait_for(
      lock, std::chrono::seconds(timeout_seconds),
      [this] { return video_done_received_; });
  if (!done) {
    std::cerr << "Timed out waiting for video-done after "
              << timeout_seconds << "s" << std::endl;
  }
  return done;
}

bool GstCamera::SetZoomLevel(float zoom) {
  if (zoom_level_ == zoom) {
    return true;
  }
  if (max_zoom_level_ < zoom) {
    std::cerr << "zoom level(" << zoom << ") is over the max-zoom level("
              << max_zoom_level_ << ")" << std::endl;
    return false;
  }
  if (min_zoom_level_ > zoom) {
    std::cerr << "zoom level(" << zoom << ") is under the min-zoom level("
              << min_zoom_level_ << ")" << std::endl;
    return false;
  }

  g_object_set(gst_.camerabin, "zoom", zoom, NULL);
  zoom_level_ = zoom;
  return true;
}

// ---------------------------------------------------------------------------
//  V4L2 direct device access
// ---------------------------------------------------------------------------

void GstCamera::OpenV4l2Device() {
  if (v4l2_fd_ >= 0) {
    return;  // Already open.
  }

  // Try to get the device path from camerabin's camera-source.
  std::string device_path = "/dev/video0";
  if (gst_.camerabin) {
    GstElement* camera_src = nullptr;
    g_object_get(gst_.camerabin, "camera-source", &camera_src, NULL);
    if (camera_src) {
      // The camera-source wraps v4l2src.  Try to find the v4l2src
      // inside.  If camerabin uses a wrappercamerabinsrc the actual
      // v4l2src is a child element.
      GstElement* v4l2src = nullptr;
      if (GST_IS_BIN(camera_src)) {
        GstIterator* iter = gst_bin_iterate_elements(GST_BIN(camera_src));
        GValue item = G_VALUE_INIT;
        while (gst_iterator_next(iter, &item) == GST_ITERATOR_OK) {
          auto* elem = GST_ELEMENT(g_value_get_object(&item));
          gchar* factory_name = nullptr;
          auto* factory = gst_element_get_factory(elem);
          if (factory) {
            factory_name =
                const_cast<gchar*>(gst_plugin_feature_get_name(
                    GST_PLUGIN_FEATURE(factory)));
          }
          if (factory_name && g_strcmp0(factory_name, "v4l2src") == 0) {
            v4l2src = elem;
            g_value_unset(&item);
            break;
          }
          g_value_unset(&item);
        }
        gst_iterator_free(iter);
      } else {
        // camera-source might itself be a v4l2src.
        auto* factory = gst_element_get_factory(camera_src);
        if (factory) {
          auto* name = gst_plugin_feature_get_name(GST_PLUGIN_FEATURE(factory));
          if (name && g_strcmp0(name, "v4l2src") == 0) {
            v4l2src = camera_src;
          }
        }
      }

      if (v4l2src) {
        gchar* dev = nullptr;
        g_object_get(v4l2src, "device", &dev, NULL);
        if (dev) {
          device_path = dev;
          g_free(dev);
        }
      }
      gst_object_unref(camera_src);
    }
  }

  v4l2_fd_ = open(device_path.c_str(), O_RDWR);
  if (v4l2_fd_ < 0) {
    std::cerr << "Failed to open V4L2 device: " << device_path << std::endl;
  } else {
    std::cout << "Opened V4L2 device: " << device_path << std::endl;
  }
}

void GstCamera::CloseV4l2Device() {
  if (v4l2_fd_ >= 0) {
    close(v4l2_fd_);
    v4l2_fd_ = -1;
  }
}

bool GstCamera::V4l2SetCtrl(uint32_t cid, int32_t value) {
  if (v4l2_fd_ < 0) return false;

  struct v4l2_control ctrl;
  ctrl.id = cid;
  ctrl.value = value;
  if (ioctl(v4l2_fd_, VIDIOC_S_CTRL, &ctrl) < 0) {
    std::cerr << "V4L2 VIDIOC_S_CTRL failed for CID 0x" << std::hex << cid
              << std::dec << ": " << strerror(errno) << std::endl;
    return false;
  }
  return true;
}

bool GstCamera::V4l2GetCtrl(uint32_t cid, int32_t& value) {
  if (v4l2_fd_ < 0) return false;

  struct v4l2_control ctrl;
  ctrl.id = cid;
  if (ioctl(v4l2_fd_, VIDIOC_G_CTRL, &ctrl) < 0) {
    std::cerr << "V4L2 VIDIOC_G_CTRL failed for CID 0x" << std::hex << cid
              << std::dec << ": " << strerror(errno) << std::endl;
    return false;
  }
  value = ctrl.value;
  return true;
}

bool GstCamera::V4l2QueryCtrl(uint32_t cid, int32_t& min, int32_t& max,
                               int32_t& step, int32_t& default_val) {
  if (v4l2_fd_ < 0) return false;

  struct v4l2_queryctrl qctrl;
  memset(&qctrl, 0, sizeof(qctrl));
  qctrl.id = cid;
  if (ioctl(v4l2_fd_, VIDIOC_QUERYCTRL, &qctrl) < 0) {
    return false;  // Control not supported — not an error.
  }
  if (qctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
    return false;
  }
  min = qctrl.minimum;
  max = qctrl.maximum;
  step = qctrl.step;
  default_val = qctrl.default_value;
  return true;
}

// ---------------------------------------------------------------------------
//  Exposure controls
// ---------------------------------------------------------------------------

bool GstCamera::SetExposureMode(const std::string& mode) {
  OpenV4l2Device();
  if (v4l2_fd_ < 0) {
    // No V4L2 access — accept silently.
    return true;
  }

  if (mode == "auto") {
    // Most UVC webcams only expose menu entries 1 (Manual) and
    // 3 (Aperture Priority).  "Aperture Priority" lets the camera
    // auto-adjust exposure time — which is "auto" in Flutter terms.
    // Try it first; fall back to V4L2_EXPOSURE_AUTO (0) for cameras
    // that support the full enum.
    if (V4l2SetCtrl(V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_APERTURE_PRIORITY)) {
      return true;
    }
    return V4l2SetCtrl(V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_AUTO);
  } else if (mode == "locked") {
    // Switch to manual exposure.  The current absolute value is retained,
    // effectively "locking" the exposure at its current level.
    return V4l2SetCtrl(V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL);
  }
  std::cerr << "Unknown exposure mode: " << mode << std::endl;
  return false;
}

double GstCamera::GetMinExposureOffset() {
  OpenV4l2Device();
  int32_t min = 0, max = 0, step = 0, def = 0;
  if (V4l2QueryCtrl(V4L2_CID_EXPOSURE_ABSOLUTE, min, max, step, def)) {
    // Convert V4L2 exposure units (100µs) to an EV-like offset.
    // We define offset relative to the default value:
    //   min_offset = log2(min / default)
    if (def > 0 && min > 0) {
      return std::log2(static_cast<double>(min) / def);
    }
  }
  return 0.0;
}

double GstCamera::GetMaxExposureOffset() {
  OpenV4l2Device();
  int32_t min = 0, max = 0, step = 0, def = 0;
  if (V4l2QueryCtrl(V4L2_CID_EXPOSURE_ABSOLUTE, min, max, step, def)) {
    if (def > 0 && max > 0) {
      return std::log2(static_cast<double>(max) / def);
    }
  }
  return 0.0;
}

double GstCamera::GetExposureOffsetStepSize() {
  OpenV4l2Device();
  int32_t min = 0, max = 0, step = 0, def = 0;
  if (V4l2QueryCtrl(V4L2_CID_EXPOSURE_ABSOLUTE, min, max, step, def)) {
    // The step in V4L2 units.  Convert to approximate EV step.
    if (def > 0 && step > 0) {
      return std::log2(static_cast<double>(def + step) / def);
    }
  }
  return 0.0;
}

double GstCamera::SetExposureOffset(double offset) {
  OpenV4l2Device();
  int32_t min = 0, max = 0, step = 0, def = 0;
  if (!V4l2QueryCtrl(V4L2_CID_EXPOSURE_ABSOLUTE, min, max, step, def)) {
    return 0.0;
  }

  // First, ensure we are in manual exposure mode so the absolute
  // value takes effect.
  V4l2SetCtrl(V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL);

  // Convert the EV offset to an absolute V4L2 exposure value.
  // offset (EV) = log2(abs / default)  →  abs = default * 2^offset
  double target = (def > 0) ? def * std::pow(2.0, offset) : 1.0;
  int32_t abs_val = static_cast<int32_t>(std::round(target));

  // Clamp to the control's range.
  if (abs_val < min) abs_val = min;
  if (abs_val > max) abs_val = max;

  V4l2SetCtrl(V4L2_CID_EXPOSURE_ABSOLUTE, abs_val);

  // Read back and convert to EV offset for the actually applied value.
  int32_t actual = abs_val;
  V4l2GetCtrl(V4L2_CID_EXPOSURE_ABSOLUTE, actual);
  if (def > 0 && actual > 0) {
    return std::log2(static_cast<double>(actual) / def);
  }
  return 0.0;
}

// ---------------------------------------------------------------------------
//  Focus controls
// ---------------------------------------------------------------------------

bool GstCamera::SetFocusMode(const std::string& mode) {
  OpenV4l2Device();
  if (v4l2_fd_ < 0) {
    // No V4L2 access — accept silently.
    return true;
  }

  // Check if continuous autofocus is supported.
  int32_t min = 0, max = 0, step = 0, def = 0;
  if (!V4l2QueryCtrl(V4L2_CID_FOCUS_AUTO, min, max, step, def)) {
    // Camera doesn't support focus control (fixed-focus lens).
    // Accept silently — the Dart side will track mode.
    return true;
  }

  if (mode == "auto") {
    return V4l2SetCtrl(V4L2_CID_FOCUS_AUTO, 1);
  } else if (mode == "locked") {
    return V4l2SetCtrl(V4L2_CID_FOCUS_AUTO, 0);
  }
  std::cerr << "Unknown focus mode: " << mode << std::endl;
  return false;
}

// ---------------------------------------------------------------------------
//  Flash control
// ---------------------------------------------------------------------------

bool GstCamera::SetFlashMode(const std::string& mode) {
  // Most USB/embedded cameras do not have flash hardware.
  // Accept any mode silently so the Dart side doesn't throw.
  // If V4L2_CID_FLASH_LED_MODE were supported, we'd set it here.
  (void)mode;
  return true;
}

const uint8_t* GstCamera::GetPreviewFrameBuffer() {
  std::shared_lock<std::shared_mutex> lock(mutex_buffer_);
  if (!gst_.buffer) {
    return nullptr;
  }

  const uint32_t pixel_bytes = width_ * height_ * 4;
  gst_buffer_extract(gst_.buffer, 0, pixels_.get(), pixel_bytes);
  return reinterpret_cast<const uint8_t*>(pixels_.get());
}

// Creates a camera pipeline using camerabin.
// $ gst-launch-1.0 camerabin viewfinder-sink="videoconvert !
// video/x-raw,format=RGBA ! fakesink"
bool GstCamera::CreatePipeline() {
  gst_.pipeline = gst_pipeline_new("pipeline");
  if (!gst_.pipeline) {
    std::cerr << "Failed to create a pipeline" << std::endl;
    return false;
  }
  gst_.camerabin = gst_element_factory_make("camerabin", "camerabin");
  if (!gst_.camerabin) {
    std::cerr << "Failed to create a source" << std::endl;
    return false;
  }
  gst_.video_convert = gst_element_factory_make("videoconvert", "videoconvert");
  if (!gst_.video_convert) {
    std::cerr << "Failed to create a videoconvert" << std::endl;
    return false;
  }
  gst_.video_sink = gst_element_factory_make("fakesink", "videosink");
  if (!gst_.video_sink) {
    std::cerr << "Failed to create a videosink" << std::endl;
    return false;
  }
  gst_.output = gst_bin_new("output");
  if (!gst_.output) {
    std::cerr << "Failed to create an output" << std::endl;
    return false;
  }
  gst_.bus = gst_pipeline_get_bus(GST_PIPELINE(gst_.pipeline));
  if (!gst_.bus) {
    std::cerr << "Failed to create a bus" << std::endl;
    return false;
  }
  gst_bus_set_sync_handler(gst_.bus, HandleGstMessage, this, NULL);

  // Sets properties to fakesink to get the callback of a decoded frame.
  g_object_set(G_OBJECT(gst_.video_sink), "sync", TRUE, "qos", FALSE, NULL);
  g_object_set(G_OBJECT(gst_.video_sink), "signal-handoffs", TRUE, NULL);
  g_signal_connect(G_OBJECT(gst_.video_sink), "handoff",
                   G_CALLBACK(HandoffHandler), this);
  gst_bin_add_many(GST_BIN(gst_.output), gst_.video_convert, gst_.video_sink,
                   NULL);

  // Adds caps to the converter to convert the color format to RGBA.
  auto* caps = gst_caps_from_string("video/x-raw,format=RGBA");
  auto link_ok =
      gst_element_link_filtered(gst_.video_convert, gst_.video_sink, caps);
  gst_caps_unref(caps);
  if (!link_ok) {
    std::cerr << "Failed to link elements" << std::endl;
    return false;
  }

  auto* sinkpad = gst_element_get_static_pad(gst_.video_convert, "sink");
  auto* ghost_sinkpad = gst_ghost_pad_new("sink", sinkpad);
  gst_pad_set_active(ghost_sinkpad, TRUE);
  gst_element_add_pad(gst_.output, ghost_sinkpad);

  // Create a valve element for the video recording path.  Setting it as
  // camerabin's "video-filter" places it between the camera source and
  // the encoder/muxer — but NOT in the viewfinder path.  Toggling
  // "drop" on this element lets us pause/resume recording without
  // stopping capture or freezing the preview.
  gst_.video_valve = gst_element_factory_make("valve", "video-valve");
  if (!gst_.video_valve) {
    std::cerr << "Warning: failed to create valve element; "
              << "pause/resume will be no-ops" << std::endl;
  } else {
    g_object_set(gst_.video_valve, "drop", FALSE, NULL);
    g_object_set(gst_.camerabin, "video-filter", gst_.video_valve, NULL);
  }

  // Sets properties to camerabin.
  g_object_set(gst_.camerabin, "viewfinder-sink", gst_.output, NULL);
  gst_bin_add_many(GST_BIN(gst_.pipeline), gst_.camerabin, NULL);

  return true;
}

void GstCamera::Preroll() {
  if (!gst_.camerabin) {
    return;
  }

  auto result = gst_element_set_state(gst_.pipeline, GST_STATE_PAUSED);
  if (result == GST_STATE_CHANGE_FAILURE) {
    std::cerr << "Failed to change the state to PAUSED" << std::endl;
    return;
  }

  // Waits until the state becomes GST_STATE_PAUSED.
  if (result == GST_STATE_CHANGE_ASYNC) {
    GstState state;
    result =
        gst_element_get_state(gst_.pipeline, &state, NULL, GST_CLOCK_TIME_NONE);
    if (result == GST_STATE_CHANGE_FAILURE) {
      std::cerr << "Failed to get the current state" << std::endl;
    }
  }
}

void GstCamera::DestroyPipeline() {
  // Close V4L2 device before tearing down GStreamer elements.
  CloseV4l2Device();

  if (gst_.video_sink) {
    g_object_set(G_OBJECT(gst_.video_sink), "signal-handoffs", FALSE, NULL);
  }

  if (gst_.pipeline) {
    gst_element_set_state(gst_.pipeline, GST_STATE_NULL);
  }

  if (gst_.buffer) {
    gst_buffer_unref(gst_.buffer);
    gst_.buffer = nullptr;
  }

  if (gst_.bus) {
    gst_object_unref(gst_.bus);
    gst_.bus = nullptr;
  }

  if (gst_.pipeline) {
    gst_object_unref(gst_.pipeline);
    gst_.pipeline = nullptr;
  }

  if (gst_.camerabin) {
    gst_.camerabin = nullptr;
  }

  if (gst_.output) {
    gst_.output = nullptr;
  }

  if (gst_.video_sink) {
    gst_.video_sink = nullptr;
  }

  if (gst_.video_valve) {
    gst_.video_valve = nullptr;
  }

  if (gst_.video_convert) {
    gst_.video_convert = nullptr;
  }
}

void GstCamera::GetZoomMaxMinSize(float& max, float& min) {
  if (!gst_.pipeline || !gst_.camerabin) {
    std::cerr << "The pipeline hasn't initialized yet.";
    return;
  }

  g_object_get(gst_.camerabin, "max-zoom", &max, NULL);
  min = 1.0;
}

// static
void GstCamera::HandoffHandler(GstElement* fakesink, GstBuffer* buf,
                                GstPad* new_pad, gpointer user_data) {
  auto* self = reinterpret_cast<GstCamera*>(user_data);
  auto* caps = gst_pad_get_current_caps(new_pad);
  auto* structure = gst_caps_get_structure(caps, 0);

  int width;
  int height;
  gst_structure_get_int(structure, "width", &width);
  gst_structure_get_int(structure, "height", &height);
  gst_caps_unref(caps);
  if (width != self->width_ || height != self->height_) {
    self->width_ = width;
    self->height_ = height;
    self->pixels_.reset(new uint32_t[width * height]);
    std::cout << "Pixel buffer size: width = " << width
              << ", height = " << height << std::endl;
  }

  std::lock_guard<std::shared_mutex> lock(self->mutex_buffer_);
  if (self->gst_.buffer) {
    gst_buffer_unref(self->gst_.buffer);
    self->gst_.buffer = nullptr;
  }
  self->gst_.buffer = gst_buffer_ref(buf);
  self->stream_handler_->OnNotifyFrameDecoded();
}

// static
GstBusSyncReply GstCamera::HandleGstMessage(GstBus* bus,
                                            GstMessage* message,
                                            gpointer user_data) {
  switch (GST_MESSAGE_TYPE(message)) {
    case GST_MESSAGE_ELEMENT: {
      auto const* st = gst_message_get_structure(message);
      if (st) {
        auto* self = reinterpret_cast<GstCamera*>(user_data);
        if (gst_structure_has_name(st, "image-done") &&
            self->on_notify_captured_) {
          auto const* filename = gst_structure_get_string(st, "filename");
          self->on_notify_captured_(filename);
        } else if (gst_structure_has_name(st, "video-done")) {
          // Signal the condvar so StopVideoRecording can proceed.
          // The actual result callback is invoked by the caller after
          // it has finished switching mode — not here — so we don't
          // block the streaming thread.
          std::lock_guard<std::mutex> lock(self->video_done_mutex_);
          self->video_done_received_ = true;
          self->video_done_cv_.notify_one();
        }
      }
      break;
    }
    case GST_MESSAGE_WARNING: {
      gchar* debug;
      GError* error;
      gst_message_parse_warning(message, &error, &debug);
      g_printerr("WARNING from element %s: %s\n",
                 GST_OBJECT_NAME(message->src), error->message);
      g_printerr("Warning details: %s\n", debug);
      g_free(debug);
      g_error_free(error);
      break;
    }
    case GST_MESSAGE_ERROR: {
      gchar* debug;
      GError* error;
      gst_message_parse_error(message, &error, &debug);
      g_printerr("ERROR from element %s: %s\n",
                 GST_OBJECT_NAME(message->src), error->message);
      g_printerr("Error details: %s\n", debug);
      g_free(debug);
      g_error_free(error);
      break;
    }
    default:
      break;
  }

  gst_message_unref(message);

  return GST_BUS_DROP;
}
