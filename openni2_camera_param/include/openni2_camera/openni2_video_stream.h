#pragma once

#include "OpenNI.h"

//#include "openni2_camera/openni2_exception.h"
#include "openni2_camera/openni2_frame_listener.h"
#include "openni2_camera/openni2_video_mode.h"

#include <memory>
#include <string>
#include <vector>

namespace openni2_wrapper
   {
class OpenNI2VideoStream : public openni::VideoStream
   {
   public:
   OpenNI2VideoStream(openni::Device& device, openni::SensorType sensor_type, std::string name);
   ~OpenNI2VideoStream() = default;

   void startStream(std::shared_ptr<OpenNI2FrameListener> frame_listener);
   void stopStream(std::shared_ptr<OpenNI2FrameListener> frame_listener);
   bool isStreamStarted() const
      {
      return (started_);
      }

   const std::vector<OpenNI2VideoMode>& getSupportedVideoModes();

   protected:
   std::string name_;
   openni::SensorType sensor_type_;
   bool started_;
   /*mutable*/ std::vector<OpenNI2VideoMode> video_modes_;
   };

   }  // end of namespace openni2_wrapper
