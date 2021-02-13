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

//#include "OpenNI.h"

#include "openni2_camera/openni2_frame_listener.h"
#include "openni2_camera/openni2_timer_filter.h"

#include <sensor_msgs/image_encodings.hpp>

#include <rclcpp/rclcpp.hpp>

//#define TIME_FILTER_LENGTH 15
constexpr std::size_t TIME_FILTER_LENGTH = 15;

namespace openni2_wrapper
   {
OpenNI2FrameListener::OpenNI2FrameListener(rclcpp::Node* node)
      : callback_(0),
        user_device_timer_(false),
        timer_filter_(new OpenNI2TimerFilter(TIME_FILTER_LENGTH)),
        node_(node),
        prev_time_stamp_(node_->now())
   {
   return;
   }

void OpenNI2FrameListener::setUseDeviceTimer(bool enable)
   {
   user_device_timer_ = enable;

   if (user_device_timer_)
      timer_filter_->clear();

   return;
   }

void OpenNI2FrameListener::onNewFrame(openni::VideoStream& stream)
   {
   stream.readFrame(&m_frame);

   if (m_frame.isValid() && callback_)
      {
      sensor_msgs::msg::Image::SharedPtr image(new sensor_msgs::msg::Image);

      rclcpp::Time ros_now = node_->now();

      if (!user_device_timer_)
         {
         image->header.stamp = ros_now;

         RCLCPP_DEBUG(rclcpp::get_logger("openni2"), "Time interval between frames: %.4f ms",
                      (float)((ros_now - prev_time_stamp_).nanoseconds() / 1e6));

         prev_time_stamp_ = ros_now;
         }
      else
         {
         uint64_t device_time_in_nsec = m_frame.getTimestamp() * 1000;
         uint64_t ros_time_in_nsec = ros_now.nanoseconds();

         double time_diff;
         if (ros_time_in_nsec > device_time_in_nsec)
            {
            time_diff = (double)(ros_time_in_nsec - device_time_in_nsec) / 1e9;
            }
         else
            {
            time_diff = -(double)(device_time_in_nsec - ros_time_in_nsec) / 1e9;
            }

         timer_filter_->addSample(time_diff);

         rclcpp::Duration filtered_time_diff(0, timer_filter_->getMedian() * 1e9);

         rclcpp::Time corrected_timestamp = rclcpp::Time(device_time_in_nsec) + filtered_time_diff;

         image->header.stamp = corrected_timestamp;

         RCLCPP_DEBUG(rclcpp::get_logger("openni2"), "Time interval between frames: %.4f ms",
                      (float)((corrected_timestamp - prev_time_stamp_).nanoseconds() / 1e6));

         prev_time_stamp_ = corrected_timestamp;
         }

      image->width = m_frame.getWidth();
      image->height = m_frame.getHeight();

      std::size_t data_size = m_frame.getDataSize();

      image->data.resize(data_size);
      memcpy(&image->data[0], m_frame.getData(), data_size);

      image->is_bigendian = 0;

      const openni::VideoMode& video_mode = m_frame.getVideoMode();
      switch (video_mode.getPixelFormat())
         {
         case openni::PIXEL_FORMAT_DEPTH_1_MM:
         image->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
         image->step = sizeof(unsigned char) * 2 * image->width;
         break;

         case openni::PIXEL_FORMAT_DEPTH_100_UM:
         image->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
         image->step = sizeof(unsigned char) * 2 * image->width;
         break;

         case openni::PIXEL_FORMAT_SHIFT_9_2:
         image->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
         image->step = sizeof(unsigned char) * 2 * image->width;
         break;

         case openni::PIXEL_FORMAT_SHIFT_9_3:
         image->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
         image->step = sizeof(unsigned char) * 2 * image->width;
         break;

         case openni::PIXEL_FORMAT_RGB888:
         image->encoding = sensor_msgs::image_encodings::RGB8;
         image->step = sizeof(unsigned char) * 3 * image->width;
         break;

         case openni::PIXEL_FORMAT_YUV422:
         image->encoding = sensor_msgs::image_encodings::YUV422;
         image->step = sizeof(unsigned char) * 4 * image->width;
         break;

         case openni::PIXEL_FORMAT_GRAY8:
         image->encoding = sensor_msgs::image_encodings::MONO8;
         image->step = sizeof(unsigned char) * 1 * image->width;
         break;

         case openni::PIXEL_FORMAT_GRAY16:
         image->encoding = sensor_msgs::image_encodings::MONO16;
         image->step = sizeof(unsigned char) * 2 * image->width;
         break;

         case openni::PIXEL_FORMAT_JPEG:

      default:
         RCLCPP_ERROR(rclcpp::get_logger("openni2"), "Invalid image encoding");
         break;
         }

      callback_(image);
      }

   return;
   }

   }  // namespace openni2_wrapper
