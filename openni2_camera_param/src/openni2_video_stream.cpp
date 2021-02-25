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
