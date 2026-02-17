// Copyright 2022 Sony Group Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "gst_camera.h"

#include <chrono>
#include <iostream>

GstCamera::GstCamera(std::unique_ptr<CameraStreamHandler> handler)
    : stream_handler_(std::move(handler)) {
  gst_.pipeline = nullptr;
  gst_.camerabin = nullptr;
  gst_.video_convert = nullptr;
  gst_.video_sink = nullptr;
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
    // If paused, resume so stop-capture can be processed.
    if (is_recording_paused_) {
      g_signal_emit_by_name(gst_.camerabin, "start-capture", NULL);
      is_recording_paused_ = false;
    }
    g_signal_emit_by_name(gst_.camerabin, "stop-capture", NULL);
    WaitForVideoDone(2);
    g_object_set(gst_.camerabin, "mode", 1, NULL);
    is_recording_ = false;
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
//    Camerabin does NOT support pausing via pipeline state changes (that
//    would freeze the viewfinder/preview as well).  Instead we use
//    stop-capture to end the current segment and start-capture to begin
//    a new one when resumed.  This leaves the pipeline in PLAYING so the
//    viewfinder keeps running during the pause.
//
//    The trade-off is that camerabin overwrites the location each time
//    start-capture is called, so the "paused" segment is lost.  For a
//    proper multi-segment recording the output files would need to be
//    concatenated, but that is out of scope here.  In practice, most
//    embedded use-cases don't need pause, so we accept this limitation
//    and keep the viewfinder alive.
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

  // If paused (we issued stop-capture already for the pause), we need to
  // re-issue start-capture so camerabin is in an active capture state,
  // then immediately stop it again.  Otherwise stop-capture is a no-op.
  if (is_recording_paused_) {
    g_object_set(gst_.camerabin, "location", video_file_path_.c_str(), NULL);
    g_signal_emit_by_name(gst_.camerabin, "start-capture", NULL);
    is_recording_paused_ = false;
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

  // Reset done-flag so we can wait for the segment to finish.
  {
    std::lock_guard<std::mutex> lock(video_done_mutex_);
    video_done_received_ = false;
  }

  // Stop the current capture segment.  The pipeline stays PLAYING so the
  // viewfinder (preview) keeps running — only the recording path stops.
  g_signal_emit_by_name(gst_.camerabin, "stop-capture", NULL);

  // Wait briefly for the segment to be flushed.
  WaitForVideoDone(3);

  is_recording_paused_ = true;
  std::cout << "Video recording paused" << std::endl;
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

  // Start a new capture segment with the same file path.
  // Note: camerabin will overwrite the previous file.  For true
  // multi-segment recording, each segment would need a unique path
  // and post-processing concatenation.
  g_object_set(gst_.camerabin, "location", video_file_path_.c_str(), NULL);

  // Reset done-flag for the new segment.
  {
    std::lock_guard<std::mutex> lock(video_done_mutex_);
    video_done_received_ = false;
  }

  g_signal_emit_by_name(gst_.camerabin, "start-capture", NULL);

  is_recording_paused_ = false;
  std::cout << "Video recording resumed" << std::endl;
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
          // Signal the condvar so StopVideoRecording / PauseVideoRecording
          // can proceed.  The actual result callback is invoked by the
          // caller after it has finished switching mode — not here — so
          // we don't block the streaming thread.
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
