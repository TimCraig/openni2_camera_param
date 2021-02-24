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

#include "openni2_camera/openni2_driver.h"
#include "openni2_camera/openni2_exception.h"

#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <chrono>
#include <functional>
#include <thread>

namespace openni2_wrapper
   {
using namespace std::chrono_literals;

OpenNI2Driver::OpenNI2Driver(const rclcpp::NodeOptions& node_options)
      : Node("openni2_camera", node_options),
        device_manager_(OpenNI2DeviceManager::getSingelton()),
        data_skip_ir_counter_(0),
        data_skip_color_counter_(0),
        data_skip_depth_counter_(0),
        enable_reconnect_(false)
   {
   // Declare parameters
   depth_ir_offset_x_ = declare_parameter<double>("depth_ir_offset_x", 5.0);
   depth_ir_offset_y_ = declare_parameter<double>("depth_ir_offset_y", 4.0);

   z_offset_mm_ = declare_parameter<int>("z_offset_mm", 0);
   z_scaling_ = declare_parameter<double>("z_scaling", 1.0);

   ir_time_offset_ = declare_parameter<double>("ir_time_offset", -0.033);
   color_time_offset_ = declare_parameter<double>("color_time_offset", -0.033);
   depth_time_offset_ = declare_parameter<double>("depth_time_offset", -0.033);

   depth_registration_ = declare_parameter<bool>("depth_registration", true);
   color_depth_synchronization_ = declare_parameter<bool>("color_depth_synchronization", false);
   auto_exposure_ = declare_parameter<bool>("auto_exposure", true);
   auto_white_balance_ = declare_parameter<bool>("auto_white_balance", true);
   use_device_time_ = declare_parameter<bool>("use_device_time", true);
   exposure_ = declare_parameter<int>("exposure", 0);
   data_skip_ = declare_parameter<int>("data_skip", 0) + 1;
   enable_reconnect_ = declare_parameter<bool>("enable_reconnect", true);

   ir_frame_id_ = declare_parameter<std::string>("ir_frame_id", "openni_ir_optical_frame");
   color_frame_id_ = declare_parameter<std::string>("rgb_frame_id", "openni_rgb_optical_frame");
   depth_frame_id_ = declare_parameter<std::string>("depth_frame_id", "openni_depth_optical_frame");

   color_info_url_ = declare_parameter<std::string>("rgb_camera_info_url", "");
   ir_info_url_ = declare_parameter<std::string>("depth_camera_info_url", "");

   // Parameters to control what is published
   publish_rgb_ = declare_parameter<bool>("publish_rgb", true);
   publish_depth_raw_ = declare_parameter<bool>("publish_depth_raw", true);
   publish_depth_ = declare_parameter<bool>("publish_depth", false);
   publish_ir_ = declare_parameter<bool>("publish_ir", false);


   genVideoModeTableMap();
   std::string mode = declare_parameter<std::string>("ir_mode", "VGA_30Hz");
   if (!lookupVideoMode(mode, ir_video_mode_))
      {
      RCLCPP_ERROR(get_logger(), "Undefined IR video mode");
      }

   mode = declare_parameter<std::string>("color_mode", "VGA_30Hz");
   if (!lookupVideoMode(mode, color_video_mode_))
      {
      RCLCPP_ERROR(get_logger(), "Undefined color video mode");
      }

   mode = declare_parameter<std::string>("depth_mode", "VGA_30Hz");
   if (!lookupVideoMode(mode, depth_video_mode_))
      {
      RCLCPP_ERROR(get_logger(), "Undefined color video mode");
      }

   ir_video_mode_.pixel_format_ = PIXEL_FORMAT_GRAY16;
   color_video_mode_.pixel_format_ = PIXEL_FORMAT_RGB888;
   depth_video_mode_.pixel_format_ = PIXEL_FORMAT_DEPTH_1_MM;

   device_id_ = declare_parameter<std::string>("device_id", "#1");
   if (device_id_ == "#1")
      {
      RCLCPP_WARN(get_logger(), "device_id is not set! Using first device.");
      }

   if (enable_reconnect_)
      {
      RCLCPP_WARN_STREAM(get_logger(), "Reconnect has been enabled, only one camera Sshould be plugged into each bus");
      }
   else
      {
      RCLCPP_WARN_STREAM(get_logger(), "Reconnect has been disabled");
      }

   initialized_ = false;
   timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&OpenNI2Driver::periodic, this));

   return;
   }

void OpenNI2Driver::periodic()
   {
   if (!initialized_)
      {
      // Handle one time initialization
      initDevice();
      advertiseROSTopics();
      applyConfigToOpenNIDevice();

      // Register parameter callback
      auto pParameterhandler =
            add_on_set_parameters_callback(std::bind(&OpenNI2Driver::paramCb, this, std::placeholders::_1));
      initialized_ = true;
      }

   // Connect the streams from the device
   colorConnectCb();
   depthConnectCb();
   irConnectCb();

   if (enable_reconnect_)
      {
      monitorConnection();
      }

   return;
   }

void OpenNI2Driver::advertiseROSTopics()
   {
   // Advertise all published topics
   image_transport::ImageTransport it(shared_from_this());

   // Prevent connection callbacks from executing until we've set all the
   // publishers. Otherwise connectCb() can fire while we're advertising
   // (say) "depth/image_raw", but before we actually assign to
   // pub_depth_raw_. Then pub_depth_raw_.getNumSubscribers() returns 0,
   // and we fail to start the depth generator.
   std::lock_guard<std::mutex> lock(connect_mutex_);

   // Asus Xtion PRO does not have an RGB camera
   if (device_->hasSensor(OpenNI2Device::COLOR))
      {
      //      pub_color_ = it.advertiseCamera("rgb/image_raw", 1);
      rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
      pub_rgb_ = create_publisher<sensor_msgs::msg::Image>("rgb/image_raw", qos);
      }

   if (device_->hasSensor(OpenNI2Device::IR))
      {
      pub_ir_ = it.advertiseCamera("ir/image", 1);
      }

   if (device_->hasSensor(OpenNI2Device::DEPTH))
      {
      //   pub_depth_raw_ = it.advertiseCamera("depth/image_raw", 1);
      rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
      pub_depth_raw_ = create_publisher<sensor_msgs::msg::Image>("depth/image_raw", qos);
      pub_depth_ = it.advertiseCamera("depth/image", 1);
      pub_projector_info_ = create_publisher<sensor_msgs::msg::CameraInfo>("projector/camera_info", 1);
      }

   ////////// CAMERA INFO MANAGER

   // The camera names are set to [rgb|depth]_[serial#], e.g.
   // depth_B00367707227042B. camera_info_manager substitutes this for
   // ${NAME} in the URL.
   std::string serial_number = device_->getStringID();
   std::string color_name = "rgb_" + serial_number;
   std::string ir_name = "depth_" + serial_number;

   // Load the saved calibrations, if they exist
   color_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, color_name, color_info_url_);
   ir_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, ir_name, ir_info_url_);

   get_serial_server = create_service<openni2_camera_param_msgs::srv::GetSerial>(
         "get_serial", std::bind(&OpenNI2Driver::getSerialCb, this, std::placeholders::_1, std::placeholders::_2));

   return;
   }

void OpenNI2Driver::setIRVideoMode(const OpenNI2VideoMode& ir_video_mode)
   {
   if (device_->isIRVideoModeSupported(ir_video_mode))
      {
      if (ir_video_mode != device_->getIRVideoMode())
         {
         device_->setIRVideoMode(ir_video_mode);
         }
      }
   else
      {
      RCLCPP_ERROR_STREAM(get_logger(), "Unsupported IR video mode - " << ir_video_mode);
      }

   return;
   }

void OpenNI2Driver::setColorVideoMode(const OpenNI2VideoMode& color_video_mode)
   {
   if (device_->isColorVideoModeSupported(color_video_mode))
      {
      if (color_video_mode != device_->getColorVideoMode())
         {
         device_->setColorVideoMode(color_video_mode);
         }
      }
   else
      {
      RCLCPP_ERROR_STREAM(get_logger(), "Unsupported color video mode - " << color_video_mode);
      }

   return;
   }

void OpenNI2Driver::setDepthVideoMode(const OpenNI2VideoMode& depth_video_mode)
   {
   if (device_->isDepthVideoModeSupported(depth_video_mode))
      {
      if (depth_video_mode != device_->getDepthVideoMode())
         {
         device_->setDepthVideoMode(depth_video_mode);
         }
      }
   else
      {
      RCLCPP_ERROR_STREAM(get_logger(), "Unsupported depth video mode - " << depth_video_mode);
      }

   return;
   }

void OpenNI2Driver::applyConfigToOpenNIDevice()
   {
   data_skip_ir_counter_ = 0;
   data_skip_color_counter_ = 0;
   data_skip_depth_counter_ = 0;

   setIRVideoMode(ir_video_mode_);
   setColorVideoMode(color_video_mode_);
   setDepthVideoMode(depth_video_mode_);

   if (device_->isImageRegistrationModeSupported())
      {
      try
         {
         // if (!config_init_ ||
         // (old_config_.depth_registration !=
         // depth_registration_))
         if (depth_registration_)
            {
            device_->setImageRegistrationMode(depth_registration_);
            }
         }
      catch (const OpenNI2Exception& exception)
         {
         RCLCPP_ERROR(get_logger(), "Could not set image registration. Reason: %s", exception.what());
         }
      }

   try
      {
      // if (!config_init_ ||
      // (old_config_.color_depth_synchronization !=
      // color_depth_synchronization_))
      if (color_depth_synchronization_)
         {
         device_->setDepthColorSync(color_depth_synchronization_);
         }
      }
   catch (const OpenNI2Exception& exception)
      {
      RCLCPP_ERROR(get_logger(), "Could not set color depth synchronization. Reason: %s", exception.what());
      }

   try
      {
      // if (!config_init_ || (old_config_.auto_exposure !=
      // auto_exposure_))
      if (auto_exposure_)
         device_->setAutoExposure(auto_exposure_);
      }
   catch (const OpenNI2Exception& exception)
      {
      RCLCPP_ERROR(get_logger(), "Could not set auto exposure. Reason: %s", exception.what());
      }

   try
      {
      // if (!config_init_ || (old_config_.auto_white_balance !=
      // auto_white_balance_))
      if (auto_white_balance_)
         device_->setAutoWhiteBalance(auto_white_balance_);
      }
   catch (const OpenNI2Exception& exception)
      {
      RCLCPP_ERROR(get_logger(), "Could not set auto white balance. Reason: %s", exception.what());
      }

   // Workaound for
   // https://github.com/ros-drivers/openni2_camera/issues/51 This is
   // only needed when any of the 3 setting change.  For simplicity this
   // check is always performed and exposure set.
   if ((!auto_exposure_ && !auto_white_balance_) && exposure_ != 0)
      {
      RCLCPP_INFO_STREAM(get_logger(), "Forcing exposure set, when auto exposure/white balance disabled");
      forceSetExposure();
      }
   else
      {
      // Setting the exposure the old way, although this should
      // not have an effect
      try
         {
         // if (!config_init_ || (old_config_.exposure !=
         // exposure_))
         device_->setExposure(exposure_);
         }
      catch (const OpenNI2Exception& exception)
         {
         RCLCPP_ERROR(get_logger(), "Could not set exposure. Reason: %s", exception.what());
         }
      }

   device_->setUseDeviceTimer(use_device_time_);

   return;
   }

void OpenNI2Driver::forceSetExposure()
   {
   int current_exposure_ = device_->getExposure();
   try
      {
      if (current_exposure_ == exposure_)
         {
         if (exposure_ < 254)
            {
            device_->setExposure(exposure_ + 1);
            }
         else
            {
            device_->setExposure(exposure_ - 1);
            }
         }

      device_->setExposure(exposure_);
      }
   catch (const OpenNI2Exception& exception)
      {
      RCLCPP_ERROR(get_logger(), "Could not set exposure. Reason: %s", exception.what());
      }

   return;
   }

void OpenNI2Driver::colorConnectCb()
   {
   if (!device_)
      {
      RCLCPP_WARN_STREAM(get_logger(), "Callback in " << __FUNCTION__ << "failed due to null device");

      return;
      }

   std::lock_guard<std::mutex> lock(connect_mutex_);

   // This does not appear to work
   //   color_subscribers_ = (pub_color_.getNumSubscribers() > 0);

   if (publish_rgb_ && !device_->isStreamStarted(OpenNI2Device::COLOR))
      {
      // Can't stream IR and RGB at the same time. Give RGB
      // preference.
      if (device_->isStreamStarted(OpenNI2Device::IR))
         {
         RCLCPP_ERROR(get_logger(), "Cannot stream RGB and IR at the same time. Streaming RGB only.");
         RCLCPP_INFO(get_logger(), "Stopping IR stream.");
         device_->stopStream(OpenNI2Device::IR);
         }

      device_->setFrameCallback(OpenNI2Device::COLOR,
                                std::bind(&OpenNI2Driver::newColorFrameCallback, this, std::placeholders::_1));

      RCLCPP_INFO(get_logger(), "Starting color stream.");
      device_->startStream(OpenNI2Device::COLOR);

      // Workaound for
      // https://github.com/ros-drivers/openni2_camera/issues/51
      if (exposure_ != 0)
         {
         RCLCPP_INFO_STREAM(get_logger(), "Exposure is set to " << exposure_ << ", forcing on color stream start");
         // delay for stream to start, before setting
         // exposure
         std::this_thread::sleep_for(100ms);
         forceSetExposure();
         }
      }
   else if (!publish_rgb_ && device_->isStreamStarted(OpenNI2Device::COLOR))
      {
      RCLCPP_INFO(get_logger(), "Stopping color stream.");
      device_->stopStream(OpenNI2Device::COLOR);

      // Start IR if it's been blocked on RGB subscribers
      //      bool need_ir = pub_ir_.getNumSubscribers() > 0;
      if (publish_ir_ && !device_->isStreamStarted(OpenNI2Device::IR))
         {
         device_->setFrameCallback(OpenNI2Device::IR,
                                   std::bind(&OpenNI2Driver::newIRFrameCallback, this, std::placeholders::_1));

         RCLCPP_INFO(get_logger(), "Starting IR stream.");
         device_->startStream(OpenNI2Device::IR);
         }
      }

   return;
   }

void OpenNI2Driver::depthConnectCb()
   {
   if (!device_)
      {
      RCLCPP_WARN_STREAM(get_logger(), "Callback in " << __FUNCTION__ << "failed due to null device");
      return;
      }

   std::lock_guard<std::mutex> lock(connect_mutex_);

   if ((publish_depth_ || publish_depth_raw_) && !device_->isStreamStarted(OpenNI2Device::DEPTH))
      {
      device_->setFrameCallback(OpenNI2Device::DEPTH,
                                std::bind(&OpenNI2Driver::newDepthFrameCallback, this, std::placeholders::_1));

      RCLCPP_INFO(get_logger(), "Starting depth stream.");
      device_->startStream(OpenNI2Device::DEPTH);
      }
   else if ((!publish_depth_ && !publish_depth_raw_) && device_->isStreamStarted(OpenNI2Device::DEPTH))
      {
      RCLCPP_INFO(get_logger(), "Stopping depth stream.");
      device_->stopStream(OpenNI2Device::DEPTH);
      }

   return;
   }

void OpenNI2Driver::irConnectCb()
   {
   if (!device_)
      {
      RCLCPP_WARN_STREAM(get_logger(), "Callback in " << __FUNCTION__ << "failed due to null device");
      return;
      }

   std::lock_guard<std::mutex> lock(connect_mutex_);

   if (publish_ir_ && !device_->isStreamStarted(OpenNI2Device::IR))
      {
      // Can't stream IR and RGB at the same time
      if (device_->isStreamStarted(OpenNI2Device::COLOR))
         {
         RCLCPP_ERROR(get_logger(), "Cannot stream RGB and IR at the same time. Streaming RGB only.");
         }
      else
         {
         device_->setFrameCallback(OpenNI2Device::IR,
                                   std::bind(&OpenNI2Driver::newIRFrameCallback, this, std::placeholders::_1));

         RCLCPP_INFO(get_logger(), "Starting IR stream.");
         device_->startStream(OpenNI2Device::IR);
         }
      }
   else if (!publish_ir_ && device_->isStreamStarted(OpenNI2Device::IR))
      {
      RCLCPP_INFO(get_logger(), "Stopping IR stream.");
      device_->stopStream(OpenNI2Device::IR);
      }

   return;
   }

void OpenNI2Driver::newIRFrameCallback(sensor_msgs::msg::Image::SharedPtr image)
   {
   if (!rclcpp::ok())
      {
      // Don't publish to invalid publishers
      return;
      }

   // std::cout << "newIRFrameCallback - counter=" << data_skip_ir_counter_ << " data_skip_=" << data_skip_ << "
   // ir_subscribers_=" << ir_subscribers_ << std::endl;
   if (((++data_skip_ir_counter_) % data_skip_) == 0)
      {
      data_skip_ir_counter_ = 0;

      if (publish_ir_)
         {
         image->header.frame_id = ir_frame_id_;
         image->header.stamp = rclcpp::Time(image->header.stamp) + rclcpp::Duration(ir_time_offset_ / 1e9);
         std::cout << "Publishing IR Camera Info (" << image->width << ", " << image->height << ")" << std::endl;
         pub_ir_.publish(image, getIRCameraInfo(image->width, image->height, image->header.stamp));
         }
      }

   return;
   }

void OpenNI2Driver::newColorFrameCallback(sensor_msgs::msg::Image::SharedPtr image)
   {
   if (!rclcpp::ok())
      {
      // Don't publish to invalid publishers
      return;
      }

   std::cout << "newColorFrameCallback - counter=" << data_skip_color_counter_ << " data_skip_=" << data_skip_
             << std::endl;
   if ((++data_skip_color_counter_) % data_skip_ == 0)
      {
      data_skip_color_counter_ = 0;

      if (publish_rgb_)
         {
         image->header.frame_id = color_frame_id_;
         image->header.stamp = rclcpp::Time(image->header.stamp) + rclcpp::Duration(color_time_offset_ / 1e9);
         std::cout << "Publishing Color Image" << std::endl;
         //         pub_color_.publish(image, getColorCameraInfo(image->width, image->height, image->header.stamp));
         pub_rgb_->publish(*image);
         }
      }

   return;
   }

void OpenNI2Driver::newDepthFrameCallback(sensor_msgs::msg::Image::SharedPtr image)
   {
   if (!rclcpp::ok())
      {
      // Don't publish to invalid publishers
      return;
      }

   if (((++data_skip_depth_counter_) % data_skip_) == 0)
      {
      data_skip_depth_counter_ = 0;

      //     if (depth_raw_subscribers_ || depth_subscribers_ || projector_info_subscribers_)
      if (publish_depth_raw_)
         {
         image->header.stamp = rclcpp::Time(image->header.stamp) + rclcpp::Duration(depth_time_offset_ / 1e9);

         // Add z offset to image data
         if (z_offset_mm_ != 0)
            {
            uint16_t* data = reinterpret_cast<uint16_t*>(&image->data[0]);
            for (unsigned int i = 0; i < image->width * image->height; ++i)
               if (data[i] != 0)
                  data[i] += z_offset_mm_;
            }

         // Scale the z of the image
         if (fabs(z_scaling_ - 1.0) > 1e-6)
            {
            uint16_t* data = reinterpret_cast<uint16_t*>(&image->data[0]);
            for (unsigned int i = 0; i < image->width * image->height; ++i)
               if (data[i] != 0)
                  data[i] = static_cast<uint16_t>(data[i] * z_scaling_);
            }

         sensor_msgs::msg::CameraInfo::SharedPtr cam_info;

         if (depth_registration_)
            {
            image->header.frame_id = color_frame_id_;
            cam_info = getColorCameraInfo(image->width, image->height, image->header.stamp);
            }
         else
            {
            image->header.frame_id = depth_frame_id_;
            cam_info = getDepthCameraInfo(image->width, image->height, image->header.stamp);
            }

         if (publish_depth_raw_)
            {
            std::cout << "Publishing Depth Raw Image" << std::endl;
            pub_depth_raw_->publish(*image);
            //            pub_depth_raw_.publish(image, cam_info);
            }
#if 0
         if (publish_depth_)
            {
            const sensor_msgs::msg::Image::SharedPtr floating_point_image = rawToFloatingPointConversion(image);
            std::cout << "Publishing Depth Floating Point Image" << std::endl;
            pub_depth_.publish(floating_point_image, cam_info);
            }
#endif
         // Projector "info" probably only useful for
         // working with disparity images
#if 0
         if (projector_info_subscribers_)
            {
            std::cout << "Publishing Projector Camera Info" << std::endl;
            pub_projector_info_->publish(*getProjectorCameraInfo(image->width, image->height, image->header.stamp));
            }
#endif
         }
      }

   return;
   }

// Methods to get calibration parameters for the various cameras
sensor_msgs::msg::CameraInfo::SharedPtr OpenNI2Driver::getDefaultCameraInfo(int width, int height, double f) const
   {
   sensor_msgs::msg::CameraInfo::SharedPtr info = std::make_shared<sensor_msgs::msg::CameraInfo>();

   info->width = width;
   info->height = height;

   // No distortion
   info->d.resize(5, 0.0);
   info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

   // Simple camera matrix: square pixels (fx = fy), principal point at
   // center
   info->k.fill(0.0);
   info->k[0] = info->k[4] = f;
   info->k[2] = (width / 2) - 0.5;
   // Aspect ratio for the camera center on Kinect (and other devices?)
   // is 4/3 This formula keeps the principal point the same in VGA and
   // SXGA modes
   info->k[5] = (width * (3. / 8.)) - 0.5;
   info->k[8] = 1.0;

   // No separate rectified image plane, so R = I
   info->r.fill(0.0);
   info->r[0] = info->r[4] = info->r[8] = 1.0;

   // Then P=K(I|0) = (K|0)
   info->p.fill(0.0);
   info->p[0] = info->p[5] = f;  // fx, fy
   info->p[2] = info->k[2];  // cx
   info->p[6] = info->k[5];  // cy
   info->p[10] = 1.0;

   return (info);
   }

/// @todo Use binning/ROI properly in publishing camera infos
sensor_msgs::msg::CameraInfo::SharedPtr OpenNI2Driver::getColorCameraInfo(int width, int height,
                                                                          rclcpp::Time time) const
   {
   sensor_msgs::msg::CameraInfo::SharedPtr info;

   if (color_info_manager_->isCalibrated())
      {
      info = std::make_shared<sensor_msgs::msg::CameraInfo>(color_info_manager_->getCameraInfo());
      if (info->width != width)
         {
         // Use uncalibrated values
         RCLCPP_WARN_ONCE(get_logger(),
                          "Image resolution doesn't match calibration of the "
                          "RGB camera. Using default parameters.");
         info = getDefaultCameraInfo(width, height, device_->getStreamFocalLength(OpenNI2Device::COLOR, height));
         }
      }
   else
      {
      // If uncalibrated, fill in default values
      info = getDefaultCameraInfo(width, height, device_->getStreamFocalLength(OpenNI2Device::COLOR, height));
      }

   // Fill in header
   info->header.stamp = time;
   info->header.frame_id = color_frame_id_;

   return (info);
   }

sensor_msgs::msg::CameraInfo::SharedPtr OpenNI2Driver::getIRCameraInfo(int width, int height, rclcpp::Time time) const
   {
   sensor_msgs::msg::CameraInfo::SharedPtr info;

   if (ir_info_manager_->isCalibrated())
      {
      info = std::make_shared<sensor_msgs::msg::CameraInfo>(ir_info_manager_->getCameraInfo());
      if (info->width != width)
         {
         // Use uncalibrated values
         RCLCPP_WARN_ONCE(get_logger(),
                          "Image resolution doesn't match calibration of the "
                          "IR camera. Using default parameters.");
         info = getDefaultCameraInfo(width, height, device_->getStreamFocalLength(OpenNI2Device::IR, height));
         }
      }
   else
      {
      // If uncalibrated, fill in default values
      info = getDefaultCameraInfo(width, height, device_->getStreamFocalLength(OpenNI2Device::DEPTH, height));
      }

   // Fill in header
   info->header.stamp = time;
   info->header.frame_id = depth_frame_id_;

   return (info);
   }

sensor_msgs::msg::CameraInfo::SharedPtr OpenNI2Driver::getDepthCameraInfo(int width, int height,
                                                                          rclcpp::Time time) const
   {
   // The depth image has essentially the same intrinsics as the IR
   // image, BUT the principal point is offset by half the size of the
   // hardware correlation window (probably 9x9 or 9x7 in 640x480 mode).
   // See http://www.ros.org/wiki/kinect_calibration/technical

   double scaling = static_cast<double>(width) / 640.0;

   sensor_msgs::msg::CameraInfo::SharedPtr info = getIRCameraInfo(width, height, time);
   info->k[2] -= depth_ir_offset_x_ * scaling;  // cx
   info->k[5] -= depth_ir_offset_y_ * scaling;  // cy
   info->p[2] -= depth_ir_offset_x_ * scaling;  // cx
   info->p[6] -= depth_ir_offset_y_ * scaling;  // cy

   /// @todo Could put this in projector frame so as to encode the
   /// baseline in P[3]
   return (info);
   }

sensor_msgs::msg::CameraInfo::SharedPtr OpenNI2Driver::getProjectorCameraInfo(int width, int height,
                                                                              rclcpp::Time time) const
   {
   // The projector info is simply the depth info with the baseline
   // encoded in the P matrix. It's only purpose is to be the "right"
   // camera info to the depth camera's "left" for processing disparity
   // images.
   sensor_msgs::msg::CameraInfo::SharedPtr info = getDepthCameraInfo(width, height, time);
   // Tx = -baseline * fx
   info->p[3] = -device_->getBaseline() * info->p[0];

   return (info);
   }

std::string OpenNI2Driver::resolveDeviceURI(const std::string& device_id)
   {
   // retrieve available device URIs, they look like this:
   // "1d27/0601@1/5" which is <vendor ID>/<product ID>@<bus
   // number>/<device number>
   std::shared_ptr<std::vector<std::string>> available_device_URIs = device_manager_->getConnectedDeviceURIs();

   // look for '#<number>' format
   if (device_id.size() > 1 && device_id[0] == '#')
      {
      std::istringstream device_number_str(device_id.substr(1));
      int device_number;
      device_number_str >> device_number;
      int device_index = device_number - 1;  // #1 refers to first device
      if (device_index >= available_device_URIs->size() || device_index < 0)
         {
         THROW_OPENNI_EXCEPTION("Invalid device number %i, there are %zu devices connected.", device_number,
                                available_device_URIs->size());
         }
      else
         {
         return (available_device_URIs->at(device_index));
         }
      }

   // look for '<bus>@<number>' format
   //   <bus>    is usb bus id, typically start at 1
   //   <number> is the device number, for consistency with
   //   openni_camera, these start at 1
   //               although 0 specifies "any device on this bus"
   else if ((device_id.size() > 1) && (device_id.find('@') != std::string::npos) &&
            (device_id.find('/') == std::string::npos))
      {
      // get index of @ character
      size_t index = device_id.find('@');
      if (index <= 0)
         {
         THROW_OPENNI_EXCEPTION("%s is not a valid device URI, you must give the bus number before the @.",
                                device_id.c_str());
         }
      if (index >= device_id.size() - 1)
         {
         THROW_OPENNI_EXCEPTION(
               "%s is not a valid device URI, you must give the device number "
               "after the @, specify 0 for any device on this bus",
               device_id.c_str());
         }

      // pull out device number on bus
      std::istringstream device_number_str(device_id.substr(index + 1));
      int device_number;
      device_number_str >> device_number;

      // reorder to @<bus>
      std::string bus = device_id.substr(0, index);
      bus.insert(0, "@");

      for (size_t i = 0; i < available_device_URIs->size(); ++i)
         {
         std::string s = (*available_device_URIs)[i];
         if (s.find(bus) != std::string::npos)
            {
            // this matches our bus, check device
            // number
            std::ostringstream ss;
            ss << bus << '/' << device_number;
            if (device_number == 0 || s.find(ss.str()) != std::string::npos)
               return (s);
            }
         }

      THROW_OPENNI_EXCEPTION("Device not found %s", device_id.c_str());
      }
   else
      {
      // check if the device id given matches a serial number of a
      // connected device
      for (std::vector<std::string>::const_iterator it = available_device_URIs->begin();
           it != available_device_URIs->end(); ++it)
         {
         try
            {
            std::string serial = device_manager_->getSerial(*it);
            if (serial.size() > 0 && device_id == serial)
               return (*it);
            }
         catch (const OpenNI2Exception& exception)
            {
            }
         }

      // everything else is treated as part of the device_URI
      bool match_found = false;
      std::string matched_uri;
      for (size_t i = 0; i < available_device_URIs->size(); ++i)
         {
         std::string s = (*available_device_URIs)[i];
         if (s.find(device_id) != std::string::npos)
            {
            if (!match_found)
               {
               matched_uri = s;
               match_found = true;
               }
            else
               {
               // more than one match
               THROW_OPENNI_EXCEPTION("Two devices match the given device id '%s': %s and %s.", device_id.c_str(),
                                      matched_uri.c_str(), s.c_str());
               }
            }
         }
      if (match_found)
         return (matched_uri);
      }

   return ("INVALID");
   }

void OpenNI2Driver::initDevice()
   {
   while (rclcpp::ok() && !device_)
      {
      try
         {
         std::string device_URI = resolveDeviceURI(device_id_);
         device_ = device_manager_->getDevice(device_URI, this);
         bus_id_ = extractBusID(device_->getUri());
         }
      catch (const OpenNI2Exception& exception)
         {
         if (!device_)
            {
            RCLCPP_INFO(get_logger(), "No matching device found.... waiting for devices. Reason: %s", exception.what());
            std::this_thread::sleep_for(3s);
            continue;
            }
         else
            {
            RCLCPP_ERROR(get_logger(), "Could not retrieve device. Reason: %s", exception.what());
            exit(-1);
            }
         }
      }

   while (rclcpp::ok() && !device_->isValid())
      {
      RCLCPP_ERROR(get_logger(), "Waiting for device initialization..");
      std::this_thread::sleep_for(100ms);
      }

   return;
   }

int OpenNI2Driver::extractBusID(const std::string& uri) const
   {
   // URI format is <vendor ID>/<product ID>@<bus number>/<device number>
   unsigned first = uri.find('@');
   unsigned last = uri.find('/', first);
   std::string bus_id = uri.substr(first + 1, last - first - 1);
   int rtn = atoi(bus_id.c_str());

   return (rtn);
   }

bool OpenNI2Driver::isConnected() const
   {
   bool ret = false;

   // TODO: The current isConnected logic assumes that there is only one
   // sensor on the bus of interest.  In the future, we could compare
   // serial numbers to make certain the same camera as been
   // re-connected.
   std::shared_ptr<std::vector<std::string>> list = device_manager_->getConnectedDeviceURIs();
   for (std::size_t i = 0; i != list->size(); ++i)
      {
      int uri_bus_id = extractBusID(list->at(i));
      if (uri_bus_id == bus_id_)
         {
         ret = true;
         }
      }

   return (ret);
   }

void OpenNI2Driver::monitorConnection()
   {
   // If the connection is lost, clean up the device.  If connected
   // and the devices is not initialized, then initialize.
   if (isConnected())
      {
      if (!device_)
         {
         RCLCPP_INFO_STREAM(get_logger(), "Detected re-connect...attempting reinit");
         try
            {
               {
               std::lock_guard<std::mutex> lock(connect_mutex_);
               std::string device_URI = resolveDeviceURI(device_id_);
               device_ = device_manager_->getDevice(device_URI, this);
               bus_id_ = extractBusID(device_->getUri());
               while (rclcpp::ok() && !device_->isValid())
                  {
                  RCLCPP_INFO(get_logger(),
                              "Waiting for device initialization, before "
                              "configuring and restarting publishers");
                  std::this_thread::sleep_for(100ms);
                  }
               }
            RCLCPP_INFO_STREAM(get_logger(), "Re-applying configuration to camera on re-init");
            // config_init_ = false;
            applyConfigToOpenNIDevice();

            // The color stream must be started in
            // order to adjust the exposure white
            // balance.
            RCLCPP_INFO_STREAM(get_logger(), "Starting color stream to adjust camera");
            colorConnectCb();

            // If auto exposure/white balance is
            // disabled, then the rbg image won't be
            // adjusted properly.  This is a work
            // around for now, but the final
            // implimentation should only allow
            // reconnection when auto exposure and
            // white balance are disabled, and FIXED
            // exposure is used instead.
            if ((!auto_exposure_ && !auto_white_balance_) && (exposure_ == 0))
               {
               RCLCPP_WARN_STREAM(get_logger(),
                                  "Reconnection should not be enabled if auto expousre"
                                  "/white balance are disabled.  Temporarily working around this issue");
               RCLCPP_WARN_STREAM(get_logger(),
                                  "Toggling exposure and white balance to auto on re-connect"
                                  ", otherwise image will be very dark");
               device_->setAutoExposure(true);
               device_->setAutoWhiteBalance(true);
               RCLCPP_INFO_STREAM(get_logger(), "Waiting for color camera to come up and adjust");
               // It takes about 2.5 seconds
               // for the camera to adjust
               std::this_thread::sleep_for(2500ms);
               RCLCPP_WARN_STREAM(get_logger(), "Resetting auto exposure and white balance to previous values");
               device_->setAutoExposure(auto_exposure_);
               device_->setAutoWhiteBalance(auto_white_balance_);
               }

            RCLCPP_INFO_STREAM(get_logger(), "Restarting publishers, if needed");
            irConnectCb();
            depthConnectCb();
            RCLCPP_INFO_STREAM(get_logger(), "Done re-initializing cameras");
            }
         catch (const OpenNI2Exception& exception)
            {
            if (!device_)
               {
               RCLCPP_INFO_STREAM(get_logger(), "Failed to re-initialize device on bus: " << bus_id_ << ", reason: "
                                                                                          << exception.what());
               }
            }
         }
      }
   else if (device_)
      {
      RCLCPP_WARN_STREAM(get_logger(), "Detected loss of connection.  Stopping all streams and resetting device");
      device_->stopAllStreams();
      device_.reset();
      }

   return;
   }

void OpenNI2Driver::genVideoModeTableMap()
   {
   video_modes_lookup_.clear();

   OpenNI2VideoMode video_mode;

   // SXGA_30Hz
   video_mode.x_resolution_ = 1280;
   video_mode.y_resolution_ = 1024;
   video_mode.frame_rate_ = 30;

   video_modes_lookup_["SXGA_30Hz"] = video_mode;

   // SXGA_15Hz
   video_mode.x_resolution_ = 1280;
   video_mode.y_resolution_ = 1024;
   video_mode.frame_rate_ = 15;

   video_modes_lookup_["SXGA_15hz"] = video_mode;

   // XGA_30Hz
   video_mode.x_resolution_ = 1280;
   video_mode.y_resolution_ = 720;
   video_mode.frame_rate_ = 30;

   video_modes_lookup_["XGA_30Hz"] = video_mode;

   // XGA_15Hz
   video_mode.x_resolution_ = 1280;
   video_mode.y_resolution_ = 720;
   video_mode.frame_rate_ = 15;

   video_modes_lookup_["XGA_15Hz"] = video_mode;

   // VGA_30Hz
   video_mode.x_resolution_ = 640;
   video_mode.y_resolution_ = 480;
   video_mode.frame_rate_ = 30;

   video_modes_lookup_["VGA_30Hz"] = video_mode;

   // VGA_25Hz
   video_mode.x_resolution_ = 640;
   video_mode.y_resolution_ = 480;
   video_mode.frame_rate_ = 25;

   video_modes_lookup_["VGA_25Hz"] = video_mode;

   // QVGA_25Hz
   video_mode.x_resolution_ = 320;
   video_mode.y_resolution_ = 240;
   video_mode.frame_rate_ = 25;

   video_modes_lookup_["QVGA_25Hz"] = video_mode;

   // QVGA_30Hz
   video_mode.x_resolution_ = 320;
   video_mode.y_resolution_ = 240;
   video_mode.frame_rate_ = 30;

   video_modes_lookup_["QVGA_30Hz"] = video_mode;

   // QVGA_60Hz
   video_mode.x_resolution_ = 320;
   video_mode.y_resolution_ = 240;
   video_mode.frame_rate_ = 60;

   video_modes_lookup_["QVGA_60Hz"] = video_mode;

   // QQVGA_25Hz
   video_mode.x_resolution_ = 160;
   video_mode.y_resolution_ = 120;
   video_mode.frame_rate_ = 25;

   video_modes_lookup_["QVGA_25Hz"] = video_mode;

   // QQVGA_30Hz
   video_mode.x_resolution_ = 160;
   video_mode.y_resolution_ = 120;
   video_mode.frame_rate_ = 30;

   video_modes_lookup_["QQVGA_30Hz"] = video_mode;

   // QQVGA_60Hz
   video_mode.x_resolution_ = 160;
   video_mode.y_resolution_ = 120;
   video_mode.frame_rate_ = 60;

   video_modes_lookup_["QQVGA_60Hz"] = video_mode;

   return;
   }

bool OpenNI2Driver::lookupVideoMode(const std::string& mode, OpenNI2VideoMode& video_mode)
   {
   bool ret = false;

   auto it = video_modes_lookup_.find(mode);
   if (it != video_modes_lookup_.end())
      {
      video_mode = it->second;
      ret = true;
      }

   return (ret);
   }

const sensor_msgs::msg::Image::SharedPtr OpenNI2Driver::rawToFloatingPointConversion(
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
      if (*in_ptr == 0 || *in_ptr == 0x7FF)
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

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(openni2_wrapper::OpenNI2Driver)
