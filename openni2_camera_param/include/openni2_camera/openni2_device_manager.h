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

#ifndef OPENNI2_DEVICE_MANAGER_H_
#define OPENNI2_DEVICE_MANAGER_H_

#include "openni2_camera/openni2_device.h"
#include "openni2_camera/openni2_device_info.h"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <ostream>
#include <string>
#include <vector>

namespace openni2_wrapper
   {
class OpenNI2DeviceListener;

class OpenNI2DeviceManager
   {
   public:
   OpenNI2DeviceManager();
   virtual ~OpenNI2DeviceManager() = default;

   static std::shared_ptr<OpenNI2DeviceManager> getSingelton();

   std::shared_ptr<std::vector<OpenNI2DeviceInfo>> getConnectedDeviceInfos() const;
   std::shared_ptr<std::vector<std::string>> getConnectedDeviceURIs() const;
   std::size_t getNumOfConnectedDevices() const;

   std::shared_ptr<OpenNI2Device> getAnyDevice(rclcpp::Node* node) const
      {
      return (std::make_shared<OpenNI2Device>("", node));
      }

   std::shared_ptr<OpenNI2Device> getDevicePtr(const std::string& device_URI, rclcpp::Node* node) const
      {
      return (std::make_shared<OpenNI2Device>(device_URI, node));
      }

   std::shared_ptr<OpenNI2Device> getDevice(const std::string& device_id, int& bus_id, rclcpp::Node* node);

   // resolves non-URI device IDs to URIs, e.g. '#1' is resolved to the URI of the first device
   std::string resolveDeviceURI(const std::string& device_id);
   std::string getSerial(const std::string& device_URI) const;
   int extractBusID(const std::string& uri) const;
   bool isConnected(int bus_id) const;

   protected:
   std::shared_ptr<OpenNI2DeviceListener> device_listener_;

   static std::shared_ptr<OpenNI2DeviceManager> singelton_;
   };

std::ostream& operator<<(std::ostream& stream, const OpenNI2DeviceManager& device_manager);

   }  // namespace openni2_wrapper

#endif
