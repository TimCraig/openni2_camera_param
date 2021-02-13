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

#include <boost/function.hpp>

#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

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

   bool hasIRSensor() const
      {
      return (openni_device_->hasSensor(openni::SENSOR_IR));
      }

   bool hasColorSensor() const
      {
      return (openni_device_->hasSensor(openni::SENSOR_COLOR));
      }

   bool hasDepthSensor() const
      {
      return (openni_device_->hasSensor(openni::SENSOR_DEPTH));
      }

   void startIRStream();
   void startColorStream();
   void startDepthStream();

   void stopAllStreams();

   void stopIRStream();
   void stopColorStream();
   void stopDepthStream();

   bool isIRStreamStarted() const
      {
      return (ir_video_started_);
      }

   bool isColorStreamStarted() const
      {
      return (color_video_started_);
      }

   bool isDepthStreamStarted() const
      {
      return (depth_video_started_);
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

   void setIRFrameCallback(FrameCallbackFunction callback)
      {
      ir_frame_listener->setCallback(callback);

      return;
      }

   void setColorFrameCallback(FrameCallbackFunction callback)
      {
      color_frame_listener->setCallback(callback);

      return;
      }

   void setDepthFrameCallback(FrameCallbackFunction callback)
      {
      depth_frame_listener->setCallback(callback);

      return;
      }

   float getIRFocalLength(int output_y_resolution) const;
   float getColorFocalLength(int output_y_resolution) const;
   float getDepthFocalLength(int output_y_resolution) const;
   float getBaseline() const;

   void setAutoExposure(bool enable);
   void setAutoWhiteBalance(bool enable);
   void setExposure(int exposure);

   bool getAutoExposure() const;
   bool getAutoWhiteBalance() const;
   int getExposure() const;

   void setUseDeviceTimer(bool enable);

   protected:
   void shutdown();

   std::shared_ptr<openni::VideoStream> getIRVideoStream() const;
   std::shared_ptr<openni::VideoStream> getColorVideoStream() const;
   std::shared_ptr<openni::VideoStream> getDepthVideoStream() const;

   std::shared_ptr<openni::Device> openni_device_;
   std::shared_ptr<openni::DeviceInfo> device_info_;

   std::shared_ptr<OpenNI2FrameListener> ir_frame_listener;
   std::shared_ptr<OpenNI2FrameListener> color_frame_listener;
   std::shared_ptr<OpenNI2FrameListener> depth_frame_listener;

   mutable std::shared_ptr<openni::VideoStream> ir_video_stream_;
   mutable std::shared_ptr<openni::VideoStream> color_video_stream_;
   mutable std::shared_ptr<openni::VideoStream> depth_video_stream_;

   mutable std::vector<OpenNI2VideoMode> ir_video_modes_;
   mutable std::vector<OpenNI2VideoMode> color_video_modes_;
   mutable std::vector<OpenNI2VideoMode> depth_video_modes_;

   bool ir_video_started_;
   bool color_video_started_;
   bool depth_video_started_;

   bool image_registration_activated_;

   bool use_device_time_;
   };

std::ostream& operator<<(std::ostream& stream, const OpenNI2Device& device);

   }  // namespace openni2_wrapper

#endif /* OPENNI_DEVICE_H */
