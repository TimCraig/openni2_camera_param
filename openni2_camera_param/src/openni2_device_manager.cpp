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

#include "openni2_camera/openni2_device_manager.h"
#include "openni2_camera/openni2_convert.h"
#include "openni2_camera/openni2_exception.h"

#include <rclcpp/rclcpp.hpp>

#include <mutex>
#include <set>
#include <string>

#include "OpenNI.h"

namespace openni2_wrapper
   {
class OpenNI2DeviceInfoComparator
   {
   public:
   bool operator()(const OpenNI2DeviceInfo& di1, const OpenNI2DeviceInfo& di2)
      {
      return (di1.uri_.compare(di2.uri_) < 0);
      }
   };

using DeviceSet = std::set<OpenNI2DeviceInfo, OpenNI2DeviceInfoComparator>;

class OpenNI2DeviceListener : public openni::OpenNI::DeviceConnectedListener,
                              public openni::OpenNI::DeviceDisconnectedListener,
                              public openni::OpenNI::DeviceStateChangedListener
   {
   public:
   OpenNI2DeviceListener()
         : openni::OpenNI::DeviceConnectedListener(),
           openni::OpenNI::DeviceDisconnectedListener(),
           openni::OpenNI::DeviceStateChangedListener()
      {
      openni::OpenNI::addDeviceConnectedListener(this);
      openni::OpenNI::addDeviceDisconnectedListener(this);
      openni::OpenNI::addDeviceStateChangedListener(this);

      // get list of currently connected devices
      openni::Array<openni::DeviceInfo> device_info_list;
      openni::OpenNI::enumerateDevices(&device_info_list);

      for (int i = 0; i < device_info_list.getSize(); ++i)
         {
         onDeviceConnected(&device_info_list[i]);
         }

      return;
      }

   ~OpenNI2DeviceListener()
      {
      openni::OpenNI::removeDeviceConnectedListener(this);
      openni::OpenNI::removeDeviceDisconnectedListener(this);
      openni::OpenNI::removeDeviceStateChangedListener(this);

      return;
      }

   virtual void onDeviceStateChanged(const openni::DeviceInfo* pInfo, openni::DeviceState state)
      {
      RCLCPP_INFO(rclcpp::get_logger("openni2"), "Device \"%s\" error state changed to %d\n", pInfo->getUri(), state);

      switch (state)
         {
         case openni::DEVICE_STATE_OK:
         onDeviceConnected(pInfo);
         break;

         case openni::DEVICE_STATE_ERROR:
         case openni::DEVICE_STATE_NOT_READY:
         case openni::DEVICE_STATE_EOF:
      default:
         onDeviceDisconnected(pInfo);
         break;
         }

      return;
      }

   virtual void onDeviceConnected(const openni::DeviceInfo* pInfo)
      {
      std::lock_guard<std::mutex> l(device_mutex_);

      const OpenNI2DeviceInfo device_info_wrapped = openni2_convert(pInfo);

      RCLCPP_INFO(rclcpp::get_logger("openni2"), "Device \"%s\" found.", pInfo->getUri());

      // make sure it does not exist in set before inserting
      device_set_.erase(device_info_wrapped);
      device_set_.insert(device_info_wrapped);

      return;
      }

   virtual void onDeviceDisconnected(const openni::DeviceInfo* pInfo)
      {
      std::lock_guard<std::mutex> l(device_mutex_);

      RCLCPP_WARN(rclcpp::get_logger("openni2"), "Device \"%s\" disconnected\n", pInfo->getUri());

      const OpenNI2DeviceInfo device_info_wrapped = openni2_convert(pInfo);
      device_set_.erase(device_info_wrapped);

      return;
      }

   std::shared_ptr<std::vector<std::string>> getConnectedDeviceURIs()
      {
      std::lock_guard<std::mutex> l(device_mutex_);

      std::shared_ptr<std::vector<std::string>> result = std::make_shared<std::vector<std::string>>();

      result->reserve(device_set_.size());

      std::set<OpenNI2DeviceInfo, OpenNI2DeviceInfoComparator>::const_iterator it;
      std::set<OpenNI2DeviceInfo, OpenNI2DeviceInfoComparator>::const_iterator it_end = device_set_.end();

      for (it = device_set_.begin(); it != it_end; ++it)
         result->push_back(it->uri_);

      return (result);
      }

   std::shared_ptr<std::vector<OpenNI2DeviceInfo>> getConnectedDeviceInfos()
      {
      std::lock_guard<std::mutex> l(device_mutex_);

      std::shared_ptr<std::vector<OpenNI2DeviceInfo>> result = std::make_shared<std::vector<OpenNI2DeviceInfo>>();

      result->reserve(device_set_.size());

      DeviceSet::const_iterator it;
      DeviceSet::const_iterator it_end = device_set_.end();

      for (it = device_set_.begin(); it != it_end; ++it)
         {
         result->push_back(*it);
         }

      return (result);
      }

   std::size_t getNumOfConnectedDevices()
      {
      std::lock_guard<std::mutex> l(device_mutex_);

      return (device_set_.size());
      }

   std::mutex device_mutex_;
   DeviceSet device_set_;
   };

//////////////////////////////////////////////////////////////////////////

std::shared_ptr<OpenNI2DeviceManager> OpenNI2DeviceManager::singelton_;

OpenNI2DeviceManager::OpenNI2DeviceManager()
   {
   openni::Status rc = openni::OpenNI::initialize();
   if (rc != openni::STATUS_OK)
      THROW_OPENNI_EXCEPTION("Initialize failed\n%s\n", openni::OpenNI::getExtendedError());

   device_listener_ = std::make_shared<OpenNI2DeviceListener>();

   return;
   }

std::shared_ptr<OpenNI2DeviceManager> OpenNI2DeviceManager::getSingelton()
   {
   if (singelton_.get() == 0)
      singelton_ = std::make_shared<OpenNI2DeviceManager>();

   return (singelton_);
   }

std::shared_ptr<std::vector<OpenNI2DeviceInfo>> OpenNI2DeviceManager::getConnectedDeviceInfos() const
   {
   return (device_listener_->getConnectedDeviceInfos());
   }

std::shared_ptr<std::vector<std::string>> OpenNI2DeviceManager::getConnectedDeviceURIs() const
   {
   return (device_listener_->getConnectedDeviceURIs());
   }

std::size_t OpenNI2DeviceManager::getNumOfConnectedDevices() const
   {
   return (device_listener_->getNumOfConnectedDevices());
   }

std::string OpenNI2DeviceManager::getSerial(const std::string& Uri) const
   {
   RCLCPP_INFO(rclcpp::get_logger("openni2_device_manager"), "OpenNI2DeviceManager::getSerial %s", Uri.c_str());

   openni::Device openni_device;
   std::string ret;

   // we need to open the device to query the serial numberS == openni::STATUS_OK))
   if ((Uri.length() > 0) && (openni_device.open(Uri.c_str()) == openni::STATUS_OK))
      {
      int serial_len = 100;
      char serial[serial_len];

      openni::Status rc = openni_device.getProperty(openni::DEVICE_PROPERTY_SERIAL_NUMBER, serial, &serial_len);
      if (rc == openni::STATUS_OK)
         ret = serial;
      else
         {
         THROW_OPENNI_EXCEPTION("Serial number query failed: %s", openni::OpenNI::getExtendedError());
         }
      // close the device again
      openni_device.close();
      }
   else
      {
      THROW_OPENNI_EXCEPTION("Device open failed: %s", openni::OpenNI::getExtendedError());
      }

   return (ret);
   }

std::string OpenNI2DeviceManager::resolveDeviceURI(const std::string& device_id)
   {
   // retrieve available device URIs, they look like this:
   // "1d27/0601@1/5" which is <vendor ID>/<product ID>@<bus
   // number>/<device number>
   std::shared_ptr<std::vector<std::string>> available_device_URIs = getConnectedDeviceURIs();

   // look for '#<number>' format
   if ((device_id.size() > 1) && (device_id[0] == '#'))
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
            if ((device_number == 0) || (s.find(ss.str()) != std::string::npos))
               {
               return (s);
               }
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
            std::string serial = getSerial(*it);
            if ((serial.size() > 0) && (device_id == serial))
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

int OpenNI2DeviceManager::extractBusID(const std::string& uri) const
   {
   // URI format is <vendor ID>/<product ID>@<bus number>/<device number>
   unsigned first = uri.find('@');
   unsigned last = uri.find('/', first);
   std::string bus_id = uri.substr(first + 1, last - first - 1);
   int rtn = atoi(bus_id.c_str());

   return (rtn);
   }

std::shared_ptr<OpenNI2Device> OpenNI2DeviceManager::getDevice(const std::string& device_id, int& bus_id,
                                                               rclcpp::Node* node)
   {
   std::string device_URI = resolveDeviceURI(device_id);
   auto device = getDevicePtr(device_URI, node);
   bus_id = extractBusID(device->getUri());

   return (device);
   }

bool OpenNI2DeviceManager::isConnected(int bus_id) const
   {
   bool ret = false;

   // TODO: The current isConnected logic assumes that there is only one
   // sensor on the bus of interest.  In the future, we could compare
   // serial numbers to make certain the same camera as been
   // re-connected.
   std::shared_ptr<std::vector<std::string>> list = getConnectedDeviceURIs();
   for (std::size_t i = 0; i != list->size(); ++i)
      {
      int uri_bus_id = extractBusID(list->at(i));
      if (uri_bus_id == bus_id)
         {
         ret = true;
         }
      }

   return (ret);
   }


std::ostream& operator<<(std::ostream& stream, const OpenNI2DeviceManager& device_manager)
   {
   std::shared_ptr<std::vector<OpenNI2DeviceInfo>> device_info = device_manager.getConnectedDeviceInfos();

   for (const auto& dev : *device_info)
      {
      stream << "Uri: " << dev.uri_ << " (Vendor: " << dev.vendor_ << ", Name: " << dev.name_
             << ", Vendor ID: " << dev.vendor_id_ << ", Product ID: " << dev.product_id_ << ")" << std::endl;
      }

   return (stream);
   }

   }  // namespace openni2_wrapper
