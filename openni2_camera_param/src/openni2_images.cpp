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

#include "openni2_camera/openni2_images.h"
#include <sensor_msgs/image_encodings.hpp>

namespace openni2_wrapper
   {
const sensor_msgs::msg::Image::SharedPtr OpenNI2DepthImageFloat::rawToFloatingPointConversion(
      const sensor_msgs::msg::Image::SharedPtr raw_image)
   {
   static const float bad_point = std::numeric_limits<float>::quiet_NaN();

   sensor_msgs::msg::Image::SharedPtr new_image = std::make_shared<sensor_msgs::msg::Image>();

   new_image->header = raw_image->header;
   new_image->width = raw_image->width;
   new_image->height = raw_image->height;
   new_image->is_bigendian = 0;
   new_image->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
   new_image->step = sizeof(float) * raw_image->width;

   std::size_t data_size = new_image->width * new_image->height;
   new_image->data.resize(data_size * sizeof(float));

   const unsigned short* in_ptr = reinterpret_cast<const unsigned short*>(&raw_image->data[0]);
   float* out_ptr = reinterpret_cast<float*>(&new_image->data[0]);

   for (std::size_t i = 0; i < data_size; ++i, ++in_ptr, ++out_ptr)
      {
      if ((*in_ptr == 0) || (*in_ptr == 0x7FF))
         {
         *out_ptr = bad_point;
         }
      else
         {
         *out_ptr = static_cast<float>(*in_ptr) / 1000.0f;
         }
      }

   return (new_image);
   }

   }  // namespace openni2_wrapper
