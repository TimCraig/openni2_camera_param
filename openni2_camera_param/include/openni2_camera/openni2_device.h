/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *      Author: Julius Kammerl (jkammerl@willowgarage.com)
 */

#pragma once

#ifndef OPENNI2_DEVICE_H
#define OPENNI2_DEVICE_H

#include "OpenNI.h"

#include "openni2_camera/openni2_exception.h"
#include "openni2_camera/openni2_frame_listener.h"
#include "openni2_camera/openni2_video_mode.h"
#include "openni2_camera/openni2_video_stream.h"

#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <array>
#include <string>
#include <vector>

namespace openni
   {
class Device;
class DeviceInfo;
class VideoStream;
class SensorInfo;
   }  // namespace openni

namespace openni2_wrapper
   {
class OpenNI2Device
   {
   public:
   OpenNI2Device(const std::string& device_URI, rclcpp::Node* node);
   virtual ~OpenNI2Device();

   // Vector indices for the different cameras and their streams
   enum StreamIndex : size_t
      {
      DEPTH,
      COLOR,
      IR
      };

   const std::string getUri() const
      {
      return (std::string(device_info_->getUri()));
      }

   const std::string getVendor() const
      {
      return (std::string(device_info_->getVendor()));
      }

   const std::string getName() const
      {
      return (std::string(device_info_->getName()));
      }

   uint16_t getUsbVendorId() const
      {
      return (device_info_->getUsbVendorId());
      }

   uint16_t getUsbProductId() const
      {
      return (device_info_->getUsbProductId());
      }

   const std::string getStringID() const;

   bool isValid() const
      {
      return ((openni_device_.get() != 0) && openni_device_->isValid());
      }

   bool hasSensor(OpenNI2Device::StreamIndex type) const
      {
      return (openni_device_->hasSensor(static_cast<openni::SensorType>(sensor_types_[type])));
      }

   void startStream(StreamIndex stream_id)
      {
      getVideoStream(stream_id)->startStream(frame_listeners_[stream_id]);
      return;
      }

   void stopStream(StreamIndex stream_id)
      {
      getVideoStream(stream_id)->stopStream(frame_listeners_[stream_id]);
      return;
      }

   void stopAllStreams();

   bool isStreamStarted(StreamIndex stream_id) const
      {
      return (getVideoStream(stream_id)->isStreamStarted());
      }

   bool isImageRegistrationModeSupported() const
      {
      return (openni_device_->isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR));
      }

   void setImageRegistrationMode(bool enabled);
   void setDepthColorSync(bool enabled);

   const OpenNI2VideoMode getIRVideoMode() const;
   const OpenNI2VideoMode getColorVideoMode() const;
   const OpenNI2VideoMode getDepthVideoMode() const;
   const std::vector<OpenNI2VideoMode>& getSupportedIRVideoModes() const;
   const std::vector<OpenNI2VideoMode>& getSupportedColorVideoModes() const;
   const std::vector<OpenNI2VideoMode>& getSupportedDepthVideoModes() const;

   bool isIRVideoModeSupported(const OpenNI2VideoMode& video_mode) const;
   bool isColorVideoModeSupported(const OpenNI2VideoMode& video_mode) const;
   bool isDepthVideoModeSupported(const OpenNI2VideoMode& video_mode) const;

   void setIRVideoMode(const OpenNI2VideoMode& video_mode);
   void setColorVideoMode(const OpenNI2VideoMode& video_mode);
   void setDepthVideoMode(const OpenNI2VideoMode& video_mode);

   void setFrameCallback(StreamIndex frame, FrameCallbackFunction callback)
      {
      frame_listeners_[frame]->setCallback(callback);
      return;
      }

   float getStreamFocalLength(StreamIndex stream_id, int output_y_resolution) const;
   float getBaseline() const;

   void setAutoExposure(bool enable);
   void setAutoWhiteBalance(bool enable);
   void setExposure(int exposure);

   bool getAutoExposure() const;
   bool getAutoWhiteBalance() const;
   int getExposure() const;

   void setUseDeviceTimer(bool enable);

   protected:
   const std::array<int, 3> sensor_types_ = {openni::SENSOR_DEPTH, openni::SENSOR_COLOR, openni::SENSOR_IR};

   void shutdown();

   std::shared_ptr<OpenNI2VideoStream> getVideoStream(StreamIndex stream) const
      {
      return (video_streams_[stream]);
      }

   std::shared_ptr<openni::Device> openni_device_;
   std::shared_ptr<openni::DeviceInfo> device_info_;

   std::vector<std::shared_ptr<OpenNI2FrameListener>> frame_listeners_;
   std::vector<std::shared_ptr<OpenNI2VideoStream>> video_streams_;

   mutable std::vector<OpenNI2VideoMode> ir_video_modes_;
   mutable std::vector<OpenNI2VideoMode> color_video_modes_;
   mutable std::vector<OpenNI2VideoMode> depth_video_modes_;

   bool image_registration_activated_;

   bool use_device_time_;
   };

std::ostream& operator<<(std::ostream& stream, const OpenNI2Device& device);

   }  // namespace openni2_wrapper

#endif /* OPENNI_DEVICE_H */
