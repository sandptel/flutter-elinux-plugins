// Copyright 2022 Sony Group Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef PACKAGES_CAMERA_CAMERA_ELINUX_GST_CAMERA_H_
#define PACKAGES_CAMERA_CAMERA_ELINUX_GST_CAMERA_H_

#include <gst/gst.h>

#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>

#include "camera_stream_handler.h"

class GstCamera {
 public:
  using OnNotifyCaptured =
      std::function<void(const std::string& captured_file_path)>;

  GstCamera(std::unique_ptr<CameraStreamHandler> handler);
  ~GstCamera();

  static void GstLibraryLoad();
  static void GstLibraryUnload();

  bool Play();
  bool Pause();
  bool Stop();

  void TakePicture(OnNotifyCaptured on_notify_captured);

  bool StartVideoRecording(const std::string& file_path);
  void StopVideoRecording(OnNotifyCaptured on_video_done);
  bool PauseVideoRecording();
  bool ResumeVideoRecording();

  bool IsRecording() const { return is_recording_; }
  bool IsRecordingPaused() const { return is_recording_paused_; }

  bool SetZoomLevel(float zoom);
  float GetMaxZoomLevel() const { return max_zoom_level_; };
  float GetMinZoomLevel() const { return min_zoom_level_; };

  const uint8_t* GetPreviewFrameBuffer();
  int32_t GetPreviewWidth() const { return width_; };
  int32_t GetPreviewHeight() const { return height_; };

  // ---------------------------------------------------------------------------
  //  Exposure controls
  // ---------------------------------------------------------------------------
  bool SetExposureMode(const std::string& mode);
  double GetMinExposureOffset();
  double GetMaxExposureOffset();
  double GetExposureOffsetStepSize();
  double SetExposureOffset(double offset);

  // ---------------------------------------------------------------------------
  //  Focus controls
  // ---------------------------------------------------------------------------
  bool SetFocusMode(const std::string& mode);

  // ---------------------------------------------------------------------------
  //  Flash control
  // ---------------------------------------------------------------------------
  bool SetFlashMode(const std::string& mode);

 private:
  struct GstCameraElements {
    GstElement* pipeline;
    GstElement* camerabin;
    GstElement* video_convert;
    GstElement* video_sink;
    GstElement* video_valve;
    GstElement* output;
    GstBus* bus;
    GstBuffer* buffer;
  };

  static void HandoffHandler(GstElement* fakesink, GstBuffer* buf,
                              GstPad* new_pad, gpointer user_data);
  static GstBusSyncReply HandleGstMessage(GstBus* bus, GstMessage* message,
                                          gpointer user_data);

  bool CreatePipeline();
  void DestroyPipeline();
  void Preroll();
  void GetZoomMaxMinSize(float& max, float& min);

  // Waits for the "video-done" bus message up to |timeout_seconds|.
  // Returns true if the message was received, false on timeout.
  bool WaitForVideoDone(int timeout_seconds);

  // V4L2 direct device access for controls not exposed by camerabin.
  void OpenV4l2Device();
  void CloseV4l2Device();
  bool V4l2SetCtrl(uint32_t cid, int32_t value);
  bool V4l2GetCtrl(uint32_t cid, int32_t& value);
  bool V4l2QueryCtrl(uint32_t cid, int32_t& min, int32_t& max,
                     int32_t& step, int32_t& default_val);

  GstCameraElements gst_;
  std::unique_ptr<uint32_t> pixels_;
  int32_t width_ = -1;
  int32_t height_ = -1;
  std::shared_mutex mutex_buffer_;
  std::unique_ptr<CameraStreamHandler> stream_handler_ = nullptr;
  float max_zoom_level_ = 1.0f;
  float min_zoom_level_ = 1.0f;
  float zoom_level_ = 1.0f;
  int captured_count_ = 0;

  // Video recording state.
  bool is_recording_ = false;
  bool is_recording_paused_ = false;
  std::string video_file_path_;

  // Callback for image capture completion.
  OnNotifyCaptured on_notify_captured_ = nullptr;

  // Synchronisation: StopVideoRecording waits for the "video-done" bus
  // message so the muxer can flush all remaining frames before we switch
  // mode back and deliver the result.
  std::mutex video_done_mutex_;
  std::condition_variable video_done_cv_;
  bool video_done_received_ = false;

  // V4L2 file descriptor for direct camera control access.
  int v4l2_fd_ = -1;
};

#endif  // PACKAGES_CAMERA_CAMERA_ELINUX_GST_CAMERA_H_
