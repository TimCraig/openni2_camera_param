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

#ifndef OPENNI2_VIDEO_MODE_H_
#define OPENNI2_VIDEO_MODE_H_

#include <cstddef>
#include <map>
#include <ostream>

namespace openni2_wrapper
   {
// copied from OniEnums.h
enum PixelFormat
   {
   PIXEL_FORMAT_UNINITIALIZED = 0,

   // Depth
   PIXEL_FORMAT_DEPTH_1_MM = 100,
   PIXEL_FORMAT_DEPTH_100_UM = 101,
   PIXEL_FORMAT_SHIFT_9_2 = 102,
   PIXEL_FORMAT_SHIFT_9_3 = 103,

   // Color
   PIXEL_FORMAT_RGB888 = 200,
   PIXEL_FORMAT_YUV422 = 201,
   PIXEL_FORMAT_GRAY8 = 202,
   PIXEL_FORMAT_GRAY16 = 203,
   PIXEL_FORMAT_JPEG = 204,
   };

struct OpenNI2VideoMode
   {
   OpenNI2VideoMode() = default;

   OpenNI2VideoMode(std::size_t x_resolution, std::size_t y_resolution, double frame_rate,
                    PixelFormat pixel_format = PIXEL_FORMAT_UNINITIALIZED)
         : x_resolution_{x_resolution},
           y_resolution_{y_resolution},
           frame_rate_{frame_rate},
           pixel_format_{pixel_format}
      {
      return;
      }

   ~OpenNI2VideoMode() = default;

   std::size_t x_resolution_;
   std::size_t y_resolution_;
   double frame_rate_;
   PixelFormat pixel_format_;
   };

std::ostream& operator<<(std::ostream& stream, const OpenNI2VideoMode& video_mode);

bool operator==(const OpenNI2VideoMode& video_mode_a, const OpenNI2VideoMode& video_mode_b);
inline bool operator!=(const OpenNI2VideoMode& video_mode_a, const OpenNI2VideoMode& video_mode_b)
   {
   return !(video_mode_a == video_mode_b);
   }

class OpenNI2VideoModes
   {
   public:
   OpenNI2VideoModes() = default;
   OpenNI2VideoModes(OpenNI2VideoModes&&) = default;
   OpenNI2VideoModes(const OpenNI2VideoModes&) = default;
   OpenNI2VideoModes& operator=(OpenNI2VideoModes&&) = default;
   OpenNI2VideoModes& operator=(const OpenNI2VideoModes&) = default;
   ~OpenNI2VideoModes() = default;
   ;

   bool lookupVideoMode(const std::string& mode, OpenNI2VideoMode& video_mode) const;

   protected:
   static const std::map<std::string, OpenNI2VideoMode> video_modes_;
   };

   }  // end of namespace openni2_wrapper

#endif
