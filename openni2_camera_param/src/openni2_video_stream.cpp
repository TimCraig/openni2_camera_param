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
/*
 * Feb 24, 2021
 * This file was not part of the original Willow Garage release but was created by refactoring openni2_device
 * to place variables and functions pretaining to video streams in a more logical place.  Hopefully, doing it this
 * way makes the code a bit easier to read and understand.
 *
 *     Tim Craig (TimCraig@Druai.com)
 */

#include "openni2_camera/openni2_video_stream.h"
#include "openni2_camera/openni2_convert.h"
#include "openni2_camera/openni2_exception.h"

namespace openni2_wrapper
   {
OpenNI2VideoStream::OpenNI2VideoStream(openni::Device& device, openni::SensorType sensor_type, std::string name)
      : sensor_type_{sensor_type}, name_{name}, started_(false)
   {
   if (device.hasSensor(sensor_type_))
      {
      const openni::Status rc = create(device, sensor_type_);
      if (rc != openni::STATUS_OK)
         {
         THROW_OPENNI_EXCEPTION("Couldn't create %s video stream: \n%s\n", name_.c_str(),
                                openni::OpenNI::getExtendedError());
         }
      }

   return;
   }

void OpenNI2VideoStream::startStream(std::shared_ptr<OpenNI2FrameListener> frame_listener)
   {
   setMirroringEnabled(false);
   start();
   addNewFrameListener(frame_listener.get());
   started_ = true;

   return;
   }

void OpenNI2VideoStream::stopStream(std::shared_ptr<OpenNI2FrameListener> frame_listener)
   {
   started_ = false;
   removeNewFrameListener(frame_listener.get());
   stop();

   return;
   }

const std::vector<OpenNI2VideoMode>& OpenNI2VideoStream::getSupportedVideoModes()
   {
   video_modes_.clear();
   const openni::SensorInfo& sensor_info = getSensorInfo();
   video_modes_ = openni2_convert(sensor_info.getSupportedVideoModes());


   return (video_modes_);
   }

   }  // end of namespace openni2_wrapper
