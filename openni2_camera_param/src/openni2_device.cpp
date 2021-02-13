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

#include <boost/algorithm/string/replace.hpp>
#include <boost/lexical_cast.hpp>

#include "openni2_camera/openni2_convert.h"
#include "openni2_camera/openni2_device.h"
#include "openni2_camera/openni2_exception.h"

#include <string>
#include <memory>

namespace openni2_wrapper
   {
OpenNI2Device::OpenNI2Device(const std::string& device_URI, rclcpp::Node* node)
      : openni_device_(),
        ir_video_started_(false),
        color_video_started_(false),
        depth_video_started_(false),
        image_registration_activated_(false),
        use_device_time_(false)
   {
   openni::Status rc = openni::OpenNI::initialize();
   if (rc != openni::STATUS_OK)
      THROW_OPENNI_EXCEPTION("Initialize failed\n%s\n", openni::OpenNI::getExtendedError());

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
      THROW_OPENNI_EXCEPTION("Device open failed\n%s\n", openni::OpenNI::getExtendedError());

   device_info_ = std::make_shared<openni::DeviceInfo>();
   *device_info_ = openni_device_->getDeviceInfo();

   ir_frame_listener = std::make_shared<OpenNI2FrameListener>(node);
   color_frame_listener = std::make_shared<OpenNI2FrameListener>(node);
   depth_frame_listener = std::make_shared<OpenNI2FrameListener>(node);

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
   std::string ID_str = getName() + "_" + getVendor();

   boost::replace_all(ID_str, "/", "");
   boost::replace_all(ID_str, ".", "");
   boost::replace_all(ID_str, "@", "");

   return (ID_str);
   }

float OpenNI2Device::getIRFocalLength(int output_y_resolution) const
   {
   float focal_length = 0.0f;

   std::shared_ptr<openni::VideoStream> stream = getIRVideoStream();
   if (stream)
      {
      focal_length = (float)output_y_resolution / (2 * tan(stream->getVerticalFieldOfView() / 2));
      }

   return (focal_length);
   }

float OpenNI2Device::getColorFocalLength(int output_y_resolution) const
   {
   float focal_length = 0.0f;

   std::shared_ptr<openni::VideoStream> stream = getColorVideoStream();
   if (stream)
      {
      focal_length = (float)output_y_resolution / (2 * tan(stream->getVerticalFieldOfView() / 2));
      }

   return (focal_length);
   }

float OpenNI2Device::getDepthFocalLength(int output_y_resolution) const
   {
   float focal_length = 0.0f;

   std::shared_ptr<openni::VideoStream> stream = getDepthVideoStream();
   if (stream)
      {
      focal_length = static_cast<float>(output_y_resolution / (2 * tan(stream->getVerticalFieldOfView() / 2)));
      }

   return (focal_length);
   }

float OpenNI2Device::getBaseline() const
   {
   float baseline_meters = 0.075f;
   std::shared_ptr<openni::VideoStream> stream = getDepthVideoStream();

   if (stream && stream->isPropertySupported(XN_STREAM_PROPERTY_EMITTER_DCMOS_DISTANCE))
      {
      double baseline_cm;
      // Device specifimec -- from PS1080.h
      stream->getProperty(XN_STREAM_PROPERTY_EMITTER_DCMOS_DISTANCE, &baseline_cm);
      baseline_meters = static_cast<float>(baseline_cm * 0.01);  // baseline from cm -> meters
      }

   return (baseline_meters);
   }

bool OpenNI2Device::isIRVideoModeSupported(const OpenNI2VideoMode& video_mode) const
   {
   getSupportedIRVideoModes();

   bool supported = false;

   std::vector<OpenNI2VideoMode>::const_iterator it = ir_video_modes_.begin();
   std::vector<OpenNI2VideoMode>::const_iterator it_end = ir_video_modes_.end();
   while (it != it_end && !supported)
      {
      supported = (*it == video_mode);
      ++it;
      }

   return (supported);
   }

bool OpenNI2Device::isColorVideoModeSupported(const OpenNI2VideoMode& video_mode) const
   {
   getSupportedColorVideoModes();

   bool supported = false;

   std::vector<OpenNI2VideoMode>::const_iterator it = color_video_modes_.begin();
   std::vector<OpenNI2VideoMode>::const_iterator it_end = color_video_modes_.end();
   while (it != it_end && !supported)
      {
      supported = (*it == video_mode);
      ++it;
      }

   return (supported);
   }

bool OpenNI2Device::isDepthVideoModeSupported(const OpenNI2VideoMode& video_mode) const
   {
   getSupportedDepthVideoModes();

   bool supported = false;

   std::vector<OpenNI2VideoMode>::const_iterator it = depth_video_modes_.begin();
   std::vector<OpenNI2VideoMode>::const_iterator it_end = depth_video_modes_.end();
   while (it != it_end && !supported)
      {
      supported = (*it == video_mode);
      ++it;
      }

   return (supported);
   }

void OpenNI2Device::startIRStream()
   {
   std::shared_ptr<openni::VideoStream> stream = getIRVideoStream();
   if (stream)
      {
      stream->setMirroringEnabled(false);
      stream->start();
      stream->addNewFrameListener(ir_frame_listener.get());
      ir_video_started_ = true;
      }

   return;
   }

void OpenNI2Device::startColorStream()
   {
   std::shared_ptr<openni::VideoStream> stream = getColorVideoStream();
   if (stream)
      {
      stream->setMirroringEnabled(false);
      stream->start();
      stream->addNewFrameListener(color_frame_listener.get());
      color_video_started_ = true;
      }

   return;
   }

void OpenNI2Device::startDepthStream()
   {
   std::shared_ptr<openni::VideoStream> stream = getDepthVideoStream();
   if (stream)
      {
      stream->setMirroringEnabled(false);
      stream->start();
      stream->addNewFrameListener(depth_frame_listener.get());
      depth_video_started_ = true;
      }

   return;
   }

void OpenNI2Device::stopAllStreams()
   {
   stopIRStream();
   stopColorStream();
   stopDepthStream();

   return;
   }

void OpenNI2Device::stopIRStream()
   {
   if (ir_video_stream_.get() != 0)
      {
      ir_video_started_ = false;
      ir_video_stream_->removeNewFrameListener(ir_frame_listener.get());
      ir_video_stream_->stop();
      }

   return;
   }

void OpenNI2Device::stopColorStream()
   {
   if (color_video_stream_.get() != 0)
      {
      color_video_started_ = false;
      color_video_stream_->removeNewFrameListener(color_frame_listener.get());
      color_video_stream_->stop();
      }

   return;
   }

void OpenNI2Device::stopDepthStream()
   {
   if (depth_video_stream_.get() != 0)
      {
      depth_video_started_ = false;
      depth_video_stream_->removeNewFrameListener(depth_frame_listener.get());
      depth_video_stream_->stop();
      }

   return;
   }

void OpenNI2Device::shutdown()
   {
   if (ir_video_stream_.get() != 0)
      ir_video_stream_->destroy();

   if (color_video_stream_.get() != 0)
      color_video_stream_->destroy();

   if (depth_video_stream_.get() != 0)
      depth_video_stream_->destroy();
   }

const std::vector<OpenNI2VideoMode>& OpenNI2Device::getSupportedIRVideoModes() const
   {
   std::shared_ptr<openni::VideoStream> stream = getIRVideoStream();
   ir_video_modes_.clear();
   if (stream)
      {
      const openni::SensorInfo& sensor_info = stream->getSensorInfo();
      ir_video_modes_ = openni2_convert(sensor_info.getSupportedVideoModes());
      }

   return (ir_video_modes_);
   }

const std::vector<OpenNI2VideoMode>& OpenNI2Device::getSupportedColorVideoModes() const
   {
   std::shared_ptr<openni::VideoStream> stream = getColorVideoStream();
   color_video_modes_.clear();
   if (stream)
      {
      const openni::SensorInfo& sensor_info = stream->getSensorInfo();
      color_video_modes_ = openni2_convert(sensor_info.getSupportedVideoModes());
      }

   return (color_video_modes_);
   }

const std::vector<OpenNI2VideoMode>& OpenNI2Device::getSupportedDepthVideoModes() const
   {
   std::shared_ptr<openni::VideoStream> stream = getDepthVideoStream();
   depth_video_modes_.clear();
   if (stream)
      {
      const openni::SensorInfo& sensor_info = stream->getSensorInfo();
      depth_video_modes_ = openni2_convert(sensor_info.getSupportedVideoModes());
      }

   return (depth_video_modes_);
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
            THROW_OPENNI_EXCEPTION("Enabling image registration mode failed: \n%s\n",
                                   openni::OpenNI::getExtendedError());
         }
      else
         {
         openni::Status rc = openni_device_->setImageRegistrationMode(openni::IMAGE_REGISTRATION_OFF);
         if (rc != openni::STATUS_OK)
            THROW_OPENNI_EXCEPTION("Enabling image registration mode failed: \n%s\n",
                                   openni::OpenNI::getExtendedError());
         }
      }

   return;
   }

void OpenNI2Device::setDepthColorSync(bool enabled)  // throw(OpenNI2Exception)
   {
   openni::Status rc = openni_device_->setDepthColorSyncEnabled(enabled);
   if (rc != openni::STATUS_OK)
      THROW_OPENNI_EXCEPTION("Enabling depth color synchronization failed: \n%s\n", openni::OpenNI::getExtendedError());

   return;
   }

const OpenNI2VideoMode OpenNI2Device::getIRVideoMode() const  // throw (OpenNI2Exception)
   {
   OpenNI2VideoMode ret;

   std::shared_ptr<openni::VideoStream> stream = getIRVideoStream();
   if (stream)
      {
      openni::VideoMode video_mode = stream->getVideoMode();

      ret = openni2_convert(video_mode);
      }
   else
      THROW_OPENNI_EXCEPTION("Could not create video stream.");

   return (ret);
   }

const OpenNI2VideoMode OpenNI2Device::getColorVideoMode() const  // throw (OpenNI2Exception)
   {
   OpenNI2VideoMode ret;

   std::shared_ptr<openni::VideoStream> stream = getColorVideoStream();
   if (stream)
      {
      openni::VideoMode video_mode = stream->getVideoMode();

      ret = openni2_convert(video_mode);
      }
   else
      THROW_OPENNI_EXCEPTION("Could not create video stream.");

   return (ret);
   }

const OpenNI2VideoMode OpenNI2Device::getDepthVideoMode() const  // throw(OpenNI2Exception)
   {
   OpenNI2VideoMode ret;

   std::shared_ptr<openni::VideoStream> stream = getDepthVideoStream();
   if (stream)
      {
      openni::VideoMode video_mode = stream->getVideoMode();

      ret = openni2_convert(video_mode);
      }
   else
      THROW_OPENNI_EXCEPTION("Could not create video stream.");

   return (ret);
   }

void OpenNI2Device::setIRVideoMode(const OpenNI2VideoMode& video_mode)  // throw (OpenNI2Exception)
   {
   std::shared_ptr<openni::VideoStream> stream = getIRVideoStream();
   if (stream)
      {
      const openni::VideoMode videoMode = openni2_convert(video_mode);
      const openni::Status rc = stream->setVideoMode(videoMode);
      if (rc != openni::STATUS_OK)
         THROW_OPENNI_EXCEPTION("Couldn't set IR video mode: \n%s\n", openni::OpenNI::getExtendedError());
      }

   return;
   }

void OpenNI2Device::setColorVideoMode(const OpenNI2VideoMode& video_mode)  // throw (OpenNI2Exception)
   {
   std::shared_ptr<openni::VideoStream> stream = getColorVideoStream();
   if (stream)
      {
      const openni::VideoMode videoMode = openni2_convert(video_mode);
      const openni::Status rc = stream->setVideoMode(videoMode);
      if (rc != openni::STATUS_OK)
         THROW_OPENNI_EXCEPTION("Couldn't set color video mode: \n%s\n", openni::OpenNI::getExtendedError());
      }

   return;
   }

void OpenNI2Device::setDepthVideoMode(const OpenNI2VideoMode& video_mode)  // throw (OpenNI2Exception)
   {
   std::shared_ptr<openni::VideoStream> stream = getDepthVideoStream();
   if (stream)
      {
      const openni::VideoMode videoMode = openni2_convert(video_mode);
      const openni::Status rc = stream->setVideoMode(videoMode);
      if (rc != openni::STATUS_OK)
         THROW_OPENNI_EXCEPTION("Couldn't set depth video mode: \n%s\n", openni::OpenNI::getExtendedError());
      }

   return;
   }

void OpenNI2Device::setAutoExposure(bool enable)  // throw (OpenNI2Exception)
   {
   std::shared_ptr<openni::VideoStream> stream = getColorVideoStream();
   if (stream)
      {
      openni::CameraSettings* camera_seeting = stream->getCameraSettings();
      if (camera_seeting)
         {
         const openni::Status rc = camera_seeting->setAutoExposureEnabled(enable);
         if (rc != openni::STATUS_OK)
            THROW_OPENNI_EXCEPTION(
                  "Couldn't set auto exposure: "
                  "\n%s\n",
                  openni::OpenNI::getExtendedError());
         }
      }

   return;
   }

void OpenNI2Device::setAutoWhiteBalance(bool enable)  // throw (OpenNI2Exception)
   {
   std::shared_ptr<openni::VideoStream> stream = getColorVideoStream();
   if (stream)
      {
      openni::CameraSettings* camera_seeting = stream->getCameraSettings();
      if (camera_seeting)
         {
         const openni::Status rc = camera_seeting->setAutoWhiteBalanceEnabled(enable);
         if (rc != openni::STATUS_OK)
            THROW_OPENNI_EXCEPTION(
                  "Couldn't set auto white balance: "
                  "\n%s\n",
                  openni::OpenNI::getExtendedError());
         }
      }

   return;
   }

void OpenNI2Device::setExposure(int exposure)  // throw (OpenNI2Exception)
   {
   std::shared_ptr<openni::VideoStream> stream = getColorVideoStream();
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

   std::shared_ptr<openni::VideoStream> stream = getColorVideoStream();
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

   std::shared_ptr<openni::VideoStream> stream = getColorVideoStream();
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

   std::shared_ptr<openni::VideoStream> stream = getColorVideoStream();
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
   if (ir_frame_listener)
      ir_frame_listener->setUseDeviceTimer(enable);

   if (color_frame_listener)
      color_frame_listener->setUseDeviceTimer(enable);

   if (depth_frame_listener)
      depth_frame_listener->setUseDeviceTimer(enable);

   return;
   }

std::shared_ptr<openni::VideoStream> OpenNI2Device::getIRVideoStream() const  // throw (OpenNI2Exception)
   {
   if (ir_video_stream_.get() == 0)
      {
      if (hasIRSensor())
         {
         ir_video_stream_ = std::make_shared<openni::VideoStream>();

         const openni::Status rc = ir_video_stream_->create(*openni_device_, openni::SENSOR_IR);
         if (rc != openni::STATUS_OK)
            THROW_OPENNI_EXCEPTION(
                  "Couldn't create IR video stream: "
                  "\n%s\n",
                  openni::OpenNI::getExtendedError());
         }
      }

   return (ir_video_stream_);
   }

std::shared_ptr<openni::VideoStream> OpenNI2Device::getColorVideoStream() const  // throw (OpenNI2Exception)
   {
   if (color_video_stream_.get() == 0)
      {
      if (hasColorSensor())
         {
         color_video_stream_ = std::make_shared<openni::VideoStream>();

         const openni::Status rc = color_video_stream_->create(*openni_device_, openni::SENSOR_COLOR);
         if (rc != openni::STATUS_OK)
            THROW_OPENNI_EXCEPTION(
                  "Couldn't create color video "
                  "stream: \n%s\n",
                  openni::OpenNI::getExtendedError());
         }
      }

   return (color_video_stream_);
   }

std::shared_ptr<openni::VideoStream> OpenNI2Device::getDepthVideoStream() const  // throw (OpenNI2Exception)
   {
   if (depth_video_stream_.get() == 0)
      {
      if (hasDepthSensor())
         {
         depth_video_stream_ = std::make_shared<openni::VideoStream>();

         const openni::Status rc = depth_video_stream_->create(*openni_device_, openni::SENSOR_DEPTH);
         if (rc != openni::STATUS_OK)
            THROW_OPENNI_EXCEPTION(
                  "Couldn't create depth video "
                  "stream: \n%s\n",
                  openni::OpenNI::getExtendedError());
         }
      }

   return (depth_video_stream_);
   }

std::ostream& operator<<(std::ostream& stream, const OpenNI2Device& device)
   {
   stream << "Device info (" << device.getUri() << ")" << std::endl;
   stream << "   Vendor: " << device.getVendor() << std::endl;
   stream << "   Name: " << device.getName() << std::endl;
   stream << "   USB Vendor ID: " << device.getUsbVendorId() << std::endl;
   stream << "   USB Product ID: " << device.getUsbVendorId() << std::endl << std::endl;

   if (device.hasIRSensor())
      {
      stream << "IR sensor video modes:" << std::endl;
      const std::vector<OpenNI2VideoMode>& video_modes = device.getSupportedIRVideoModes();

      std::vector<OpenNI2VideoMode>::const_iterator it = video_modes.begin();
      std::vector<OpenNI2VideoMode>::const_iterator it_end = video_modes.end();
      for (; it != it_end; ++it)
         stream << "   - " << *it << std::endl;
      }
   else
      {
      stream << "No IR sensor available" << std::endl;
      }

   if (device.hasColorSensor())
      {
      stream << "Color sensor video modes:" << std::endl;
      const std::vector<OpenNI2VideoMode>& video_modes = device.getSupportedColorVideoModes();

      std::vector<OpenNI2VideoMode>::const_iterator it = video_modes.begin();
      std::vector<OpenNI2VideoMode>::const_iterator it_end = video_modes.end();
      for (; it != it_end; ++it)
         stream << "   - " << *it << std::endl;
      }
   else
      {
      stream << "No Color sensor available" << std::endl;
      }

   if (device.hasDepthSensor())
      {
      stream << "Depth sensor video modes:" << std::endl;
      const std::vector<OpenNI2VideoMode>& video_modes = device.getSupportedDepthVideoModes();

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
