#pragma once

#include "OpenNI.h"

//#include "openni2_camera/openni2_exception.h"
//#include "openni2_camera/openni2_frame_listener.h"
//#include "openni2_camera/openni2_video_mode.h"

//#include <boost/function.hpp>

//#include <cstdint>

//#include <rclcpp/rclcpp.hpp>
//#include <sensor_msgs/msg/image.hpp>

#include <string>
//#include <vector>

namespace openni2_wrapper
   {
class openni2_video_stream : public openni::VideoStream
   {
   public:
   openni2_video_stream(openni::Device& device, openni::SensorType sensor_type, std::string name);
   ~openni2_video_stream() = default;

   protected:
   std::string name_;
   openni::SensorType sensor_type_;
   };


   }  // end of namespace openni2_wrapper
