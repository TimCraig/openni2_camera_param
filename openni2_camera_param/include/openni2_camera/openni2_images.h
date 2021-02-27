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
 * Feb 27, 2021
 * This file was not part of the original Willow Garage release but was created by refactoring openni2_device
 * to place variables and functions pretaining to video streams in a more logical place.  Hopefully, doing it this
 * way makes the code a bit easier to read and understand.
 *
 *     Tim Craig (TimCraig@Druai.com)
 */

#pragma once

#ifndef OPENNI2_IMAGES_H
#define OPENNI2_IMAGES_H

#include <sensor_msgs/msg/image.hpp>

namespace openni2_wrapper
   {
class OpenNI2DepthImageFloat : public sensor_msgs::msg::Image
   {
   public:
   OpenNI2DepthImageFloat() = default;
   OpenNI2DepthImageFloat(const OpenNI2DepthImageFloat &) = default;
   OpenNI2DepthImageFloat &operator=(OpenNI2DepthImageFloat &&) = default;
   OpenNI2DepthImageFloat &operator=(const OpenNI2DepthImageFloat &) = default;
   ~OpenNI2DepthImageFloat();

   static const sensor_msgs::msg::Image::SharedPtr rawToFloatingPointConversion(
         const sensor_msgs::msg::Image::SharedPtr raw_image);

   private:
   };


   }  // namespace openni2_wrapper

#endif
