#include "openni2_camera/openni2_video_stream.h"
#include "openni2_camera/openni2_exception.h"

namespace openni2_wrapper
   {
openni2_video_stream::openni2_video_stream(openni::Device& device, openni::SensorType sensor_type, std::string name)
      : sensor_type_{sensor_type}, name_{name}
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

   }  // end of namespace openni2_wrapper
