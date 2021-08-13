/**
 * Copyright (C) 2021, Raffaello Bonghi <raffaello@rnext.it>
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its 
 *    contributors may be used to endorse or promote products derived 
 *    from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <chrono>
#include <functional>
#include <memory>

#include <sensor_msgs/msg/image.hpp>
#include "nanosaur_camera/camera_publisher.h"

#ifdef gts_default
  #define CAMERA_WIDTH gstCamera::DefaultWidth
  #define CAMERA_HEIGHT gstCamera::DefaultHeight
  #define CAMERA_FRAMERATE 120.0
#else
  #define CAMERA_WIDTH 320
  #define CAMERA_HEIGHT 240
  #define CAMERA_FRAMERATE 15.0
#endif

using namespace std::chrono_literals;

CameraPublisher::CameraPublisher() : Node("camera_publisher"), frameId("camera_optical_frame")
{
  /* create image converter */
  camera_cvt = new imageConverter();
  // Load frame_id name
  this->declare_parameter<std::string>("frame_id", frameId);
  this->get_parameter("frame_id", frameId);
  RCLCPP_INFO(this->get_logger(), "Frame ID: %s", frameId.c_str());
  // Initialize publisher
  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
  info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
  // Initialize camera
  std::string camera_device = "0";	// MIPI CSI camera by default
  this->declare_parameter<std::string>("camera.device", camera_device);
  this->get_parameter("camera.device", camera_device);
  RCLCPP_INFO(this->get_logger(), "opening camera device %s", camera_device.c_str());
  // Load configuration
  double frameRate = CAMERA_FRAMERATE;
  this->declare_parameter<double>("camera.frameRate", frameRate);
  this->get_parameter("camera.frameRate", frameRate);
  int camera_width = CAMERA_WIDTH;
  this->declare_parameter<int>("camera.width", camera_width);
  this->get_parameter("camera.width", camera_width);
  int camera_height = CAMERA_HEIGHT;
  this->declare_parameter<int>("camera.height", camera_height);
  this->get_parameter("camera.height", camera_height);

  RCLCPP_INFO(this->get_logger(), "width %d height %d - Framerate %f", camera_width, camera_height, frameRate);

  videoOptions video_options;

  std::string codec_str = "unknown";
  std::string flip_str = "";

  if( codec_str.size() != 0 )
    video_options.codec = videoOptions::CodecFromStr(codec_str.c_str());

  if( flip_str.size() != 0 )
    video_options.flipMethod = videoOptions::FlipMethodFromStr(flip_str.c_str());

  video_options.width = camera_width;
  video_options.height = camera_height;
  video_options.frameRate = (float)frameRate;
  video_options.loop = 0;
  //video_options.ioType = videoOptions::INPUT;

  /* open camera device */
  camera = videoSource::Create(camera_device.c_str(), video_options);

  if( !camera )
  {
    RCLCPP_ERROR(this->get_logger(), "failed to open camera device %s", camera_device.c_str());
  }

  /*
   * start the camera streaming
   */
  if( !camera->Open() )
  {
    RCLCPP_ERROR(this->get_logger(), "failed to start streaming video source");
  }

  /*
   * Load camera info
   */
  std::string camera_name = "camerav2";
  std::string camera_url = "package://nanosaur_camera/camera_info/camerav2.yml";
  cinfo_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, camera_name, camera_url);

  ci_ = std::make_unique<sensor_msgs::msg::CameraInfo>(cinfo_->getCameraInfo());
}

bool CameraPublisher::isStreaming()
{
  return camera->IsStreaming();
}

bool CameraPublisher::acquire()
{
  imageConverter::PixelType* nextFrame = NULL;
  // get the latest frame
  if( !camera->Capture(&nextFrame, 1000) )
  {
    RCLCPP_ERROR(this->get_logger(), "failed to capture camera frame");
    return false;
  }
  // assure correct image size
  if( !camera_cvt->Resize(camera->GetWidth(), camera->GetHeight(), imageConverter::ROSOutputFormat) )
  {
    RCLCPP_ERROR(this->get_logger(), "failed to resize camera image converter");
    return false;
  }
  // populate the message
  sensor_msgs::msg::Image img;

  if( !camera_cvt->Convert(img, imageConverter::ROSOutputFormat, nextFrame) )
  {
    RCLCPP_ERROR(this->get_logger(), "failed to convert camera frame to sensor_msgs::Image");
    return false;
  }
  auto stamp = this->get_clock()->now();
  // Add header,timestamp and frame_id
  img.header.stamp = stamp;
  img.header.frame_id = frameId;
  // Make Camera info message
  ci_->header.stamp = stamp;

  // Publish camera frame message
  image_pub_->publish(img);
  info_pub_->publish(*ci_);
  RCLCPP_DEBUG(this->get_logger(), "Published camera frame");
  return true;
}

CameraPublisher::~CameraPublisher()
{
  RCLCPP_DEBUG(this->get_logger(), "Close camera_publisher");
  camera->Close();
  camera_cvt->Free();
}
// EOF
