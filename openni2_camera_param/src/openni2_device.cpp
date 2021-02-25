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

#include <PS1080.h>  // For XN_STREAM_PROPERTY_EMITTER_DCMOS_DISTANCE property

#include "openni2_camera/openni2_convert.h"
#include "openni2_camera/openni2_device.h"
#include "openni2_camera/openni2_exception.h"

//#include <cassert>
#include <memory>
#include <regex>
#include <string>

namespace openni2_wrapper
   {
OpenNI2Device::OpenNI2Device(const std::string& device_URI, rclcpp::Node* node)
      : openni_device_(),
        image_registration_activated_(false),
        use_device_time_(false),
        stream_names_{std::string{"Depth"}, std::string{"Color"}, std::string{"IR"}}
   {
   openni::Status rc = openni::OpenNI::initialize();
   if (rc != openni::STATUS_OK)
      {
      THROW_OPENNI_EXCEPTION("Initialize failed\n%s\n", openni::OpenNI::getExtendedError());
      }

   openni_device_ = std::make_shared<openni::Device>();
   if (device_URI.length() > 0)
      {
      rc = openni_device_->open(device_URI.c_str());
      }
   else
      {
      rc = openni_device_->open(openni::ANY_DEVICE);
      }
   if (rc != openni::STATUS_OK)
      {
      THROW_OPENNI_EXCEPTION("Device open failed\n%s\n", openni::OpenNI::getExtendedError());
      }

   device_info_ = std::make_shared<openni::DeviceInfo>();
   *device_info_ = openni_device_->getDeviceInfo();

   video_streams_.push_back(
         std::make_shared<OpenNI2VideoStream>(*openni_device_, openni::SENSOR_DEPTH, stream_names_[0]));
   video_streams_.push_back(
         std::make_shared<OpenNI2VideoStream>(*openni_device_, openni::SENSOR_COLOR, stream_names_[1]));
   video_streams_.push_back(std::make_shared<OpenNI2VideoStream>(*openni_device_, openni::SENSOR_IR, stream_names_[2]));

   frame_listeners_ = {std::make_shared<OpenNI2FrameListener>(node), std::make_shared<OpenNI2FrameListener>(node),
                       std::make_shared<OpenNI2FrameListener>(node)};

   return;
   }


OpenNI2Device::~OpenNI2Device()
   {
   stopAllStreams();
   shutdown();
   openni_device_->close();

   return;
   }

const std::string OpenNI2Device::getStringID() const
   {
   std::string Raw = getName() + "_" + getVendor();

   std::regex reg("|/|\\.|@|");
   std::string ID_str = std::regex_replace(Raw, reg, "");

   return (ID_str);
   }

float OpenNI2Device::getStreamFocalLength(StreamIndex stream_id, int output_y_resolution) const
   {
   float focal_length = 0.0f;

   std::shared_ptr<openni::VideoStream> stream = getVideoStream(stream_id);
   if (stream)
      {
      focal_length = static_cast<float>(output_y_resolution / (2 * tan(stream->getVerticalFieldOfView() / 2)));
      }

   return (focal_length);
   }

float OpenNI2Device::getBaseline() const
   {
   float baseline_meters = 0.075f;
   std::shared_ptr<openni::VideoStream> stream = getVideoStream(DEPTH);

   if (stream && stream->isPropertySupported(XN_STREAM_PROPERTY_EMITTER_DCMOS_DISTANCE))
      {
      double baseline_cm;
      // Device specifimec -- from PS1080.h
      stream->getProperty(XN_STREAM_PROPERTY_EMITTER_DCMOS_DISTANCE, &baseline_cm);
      baseline_meters = static_cast<float>(baseline_cm * 0.01);  // baseline from cm -> meters
      }

   return (baseline_meters);
   }

bool OpenNI2Device::isVideoModeSupported(StreamIndex stream_id, const OpenNI2VideoMode& video_mode) const
   {
   const std::vector<OpenNI2VideoMode>& video_modes = getSupportedVideoModes(stream_id);

   bool supported = false;

   std::vector<OpenNI2VideoMode>::const_iterator it = video_modes.begin();
   std::vector<OpenNI2VideoMode>::const_iterator it_end = video_modes.end();
   while (it != it_end && !supported)
      {
      supported = (*it == video_mode);
      ++it;
      }

   return (supported);
   }

void OpenNI2Device::stopAllStreams()
   {
   stopStream(DEPTH);
   stopStream(COLOR);
   stopStream(IR);

   return;
   }

void OpenNI2Device::shutdown()
   {
#if 0
   if (ir_video_stream_.get() != 0)
      ir_video_stream_->destroy();

   if (color_video_stream_.get() != 0)
      color_video_stream_->destroy();

   if (depth_video_stream_.get() != 0)
      depth_video_stream_->destroy();
#endif
   }

void OpenNI2Device::setImageRegistrationMode(bool enabled)  // throw (OpenNI2Exception)
   {
   if (isImageRegistrationModeSupported())
      {
      image_registration_activated_ = enabled;
      if (enabled)
         {
         openni::Status rc = openni_device_->setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
         if (rc != openni::STATUS_OK)
            {
            THROW_OPENNI_EXCEPTION("Enabling image registration mode failed: \n%s\n",
                                   openni::OpenNI::getExtendedError());
            }
         }
      else
         {
         openni::Status rc = openni_device_->setImageRegistrationMode(openni::IMAGE_REGISTRATION_OFF);
         if (rc != openni::STATUS_OK)
            {
            THROW_OPENNI_EXCEPTION("Enabling image registration mode failed: \n%s\n",
                                   openni::OpenNI::getExtendedError());
            }
         }
      }

   return;
   }

void OpenNI2Device::setDepthColorSync(bool enabled)  // throw(OpenNI2Exception)
   {
   openni::Status rc = openni_device_->setDepthColorSyncEnabled(enabled);
   if (rc != openni::STATUS_OK)
      {
      THROW_OPENNI_EXCEPTION("Enabling depth color synchronization failed: \n%s\n", openni::OpenNI::getExtendedError());
      }

   return;
   }

const OpenNI2VideoMode OpenNI2Device::getVideoMode(StreamIndex stream_id) const  // throw (OpenNI2Exception)
   {
   OpenNI2VideoMode ret;

   std::shared_ptr<openni::VideoStream> stream = getVideoStream(stream_id);
   if (stream)
      {
      openni::VideoMode video_mode = stream->getVideoMode();

      ret = openni2_convert(video_mode);
      }
   else
      {
      THROW_OPENNI_EXCEPTION("Could not %s create video stream.", stream_names_[stream_id].c_str());
      }

   return (ret);
   }

void OpenNI2Device::setVideoMode(StreamIndex stream_id, const OpenNI2VideoMode& video_mode)  // throw (OpenNI2Exception)
   {
   std::shared_ptr<openni::VideoStream> stream = getVideoStream(stream_id);
   if (stream)
      {
      const openni::VideoMode videoMode = openni2_convert(video_mode);
      const openni::Status rc = stream->setVideoMode(videoMode);
      if (rc != openni::STATUS_OK)
         {
         THROW_OPENNI_EXCEPTION("Couldn't set video mode: %s\n%s\n", stream_names_[stream_id].c_str(),
                                openni::OpenNI::getExtendedError());
         }
      }

   return;
   }

void OpenNI2Device::setAutoExposure(bool enable)  // throw (OpenNI2Exception)
   {
   std::shared_ptr<openni::VideoStream> stream = getVideoStream(COLOR);
   if (stream)
      {
      openni::CameraSettings* camera_seeting = stream->getCameraSettings();
      if (camera_seeting)
         {
         const openni::Status rc = camera_seeting->setAutoExposureEnabled(enable);
         if (rc != openni::STATUS_OK)
            {
            THROW_OPENNI_EXCEPTION("Couldn't set auto exposure: \n%s\n", openni::OpenNI::getExtendedError());
            }
         }
      }

   return;
   }

void OpenNI2Device::setAutoWhiteBalance(bool enable)  // throw (OpenNI2Exception)
   {
   std::shared_ptr<openni::VideoStream> stream = getVideoStream(COLOR);
   if (stream)
      {
      openni::CameraSettings* camera_seeting = stream->getCameraSettings();
      if (camera_seeting)
         {
         const openni::Status rc = camera_seeting->setAutoWhiteBalanceEnabled(enable);
         if (rc != openni::STATUS_OK)
            {
            THROW_OPENNI_EXCEPTION("Couldn't set auto white balance: \n%s\n", openni::OpenNI::getExtendedError());
            }
         }
      }

   return;
   }

void OpenNI2Device::setExposure(int exposure)  // throw (OpenNI2Exception)
   {
   std::shared_ptr<openni::VideoStream> stream = getVideoStream(COLOR);
   if (stream)
      {
      openni::CameraSettings* camera_settings = stream->getCameraSettings();
      if (camera_settings)
         {
         const openni::Status rc = camera_settings->setExposure(exposure);
         if (rc != openni::STATUS_OK)
            THROW_OPENNI_EXCEPTION("Couldn't set exposure: \n%s\n", openni::OpenNI::getExtendedError());
         }
      }

   return;
   }

bool OpenNI2Device::getAutoExposure() const
   {
   bool ret = false;

   std::shared_ptr<openni::VideoStream> stream = getVideoStream(COLOR);
   if (stream)
      {
      openni::CameraSettings* camera_seeting = stream->getCameraSettings();
      if (camera_seeting)
         ret = camera_seeting->getAutoExposureEnabled();
      }

   return (ret);
   }

bool OpenNI2Device::getAutoWhiteBalance() const
   {
   bool ret = false;

   std::shared_ptr<openni::VideoStream> stream = getVideoStream(COLOR);
   if (stream)
      {
      openni::CameraSettings* camera_seeting = stream->getCameraSettings();
      if (camera_seeting)
         ret = camera_seeting->getAutoWhiteBalanceEnabled();
      }

   return (ret);
   }

int OpenNI2Device::getExposure() const
   {
   int ret = 0;

   std::shared_ptr<openni::VideoStream> stream = getVideoStream(COLOR);
   if (stream)
      {
      openni::CameraSettings* camera_settings = stream->getCameraSettings();
      if (camera_settings)
         ret = camera_settings->getExposure();
      }

   return (ret);
   }

void OpenNI2Device::setUseDeviceTimer(bool enable)
   {
   for (auto& listener : frame_listeners_)
      {
      listener->setUseDeviceTimer(enable);
      }

   return;
   }

std::ostream& operator<<(std::ostream& stream, const OpenNI2Device& device)
   {
   stream << "Device info (" << device.getUri() << ")" << std::endl;
   stream << "   Vendor: " << device.getVendor() << std::endl;
   stream << "   Name: " << device.getName() << std::endl;
   stream << "   USB Vendor ID: " << device.getUsbVendorId() << std::endl;
   stream << "   USB Product ID: " << device.getUsbVendorId() << std::endl << std::endl;

   if (device.hasSensor(OpenNI2Device::IR))
      {
      stream << "IR sensor video modes:" << std::endl;
      const std::vector<OpenNI2VideoMode>& video_modes = device.getSupportedVideoModes(OpenNI2Device::IR);

      std::vector<OpenNI2VideoMode>::const_iterator it = video_modes.begin();
      std::vector<OpenNI2VideoMode>::const_iterator it_end = video_modes.end();
      for (; it != it_end; ++it)
         {
         stream << "   - " << *it << std::endl;
         }
      }
   else
      {
      stream << "No IR sensor available" << std::endl;
      }

   if (device.hasSensor(OpenNI2Device::COLOR))
      {
      stream << "Color sensor video modes:" << std::endl;
      const std::vector<OpenNI2VideoMode>& video_modes = device.getSupportedVideoModes(OpenNI2Device::COLOR);

      std::vector<OpenNI2VideoMode>::const_iterator it = video_modes.begin();
      std::vector<OpenNI2VideoMode>::const_iterator it_end = video_modes.end();
      for (; it != it_end; ++it)
         stream << "   - " << *it << std::endl;
      }
   else
      {
      stream << "No Color sensor available" << std::endl;
      }

   if (device.hasSensor(OpenNI2Device::DEPTH))
      {
      stream << "Depth sensor video modes:" << std::endl;
      const std::vector<OpenNI2VideoMode>& video_modes = device.getSupportedVideoModes(OpenNI2Device::DEPTH);

      std::vector<OpenNI2VideoMode>::const_iterator it = video_modes.begin();
      std::vector<OpenNI2VideoMode>::const_iterator it_end = video_modes.end();
      for (; it != it_end; ++it)
         stream << "   - " << *it << std::endl;
      }
   else
      {
      stream << "No Depth sensor available" << std::endl;
      }

   return (stream);
   }

   }  // namespace openni2_wrapper
