// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

using std::placeholders::_1;

class Openni2Subscriber : public rclcpp::Node
  {
  public:
    Openni2Subscriber() : Node("openni2_subscriber")
      {
      subscription_ = create_subscription<sensor_msgs::msg::Image>(
        "/camera/depth/image", 10, std::bind(&Openni2Subscriber::topic_callback, this, _1));
      }

  private:
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr image) const
      {
      RCLCPP_INFO(get_logger(), "Received Image (%d,%d)\n", image->width, image->height);
      }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  };

int main(int argc, char* argv[])
  {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Openni2Subscriber>());
  rclcpp::shutdown();

  return 0;
  }
