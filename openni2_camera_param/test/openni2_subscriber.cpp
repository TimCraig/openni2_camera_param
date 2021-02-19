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

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using std::placeholders::_1;

class Openni2Subscriber : public rclcpp::Node
   {
   public:
   Openni2Subscriber() : Node("openni2_subscriber")
      {
      color_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/rgb/image_raw", 10, std::bind(&Openni2Subscriber::color_callback, this, _1));

      depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_raw", 10, std::bind(&Openni2Subscriber::depth_callback, this, _1));

      return;
      }

   private:
   void color_callback(const sensor_msgs::msg::Image::SharedPtr image) const;
   void depth_callback(const sensor_msgs::msg::Image::SharedPtr image) const;

   rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
   rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
   };

void Openni2Subscriber::color_callback(const sensor_msgs::msg::Image::SharedPtr image) const
   {
   RCLCPP_INFO(get_logger(), "Received Color Image (%d,%d)\n", image->width, image->height);

   using namespace cv_bridge;

   CvImagePtr cv_image = toCvCopy(image, "bgr8");

   cv::imshow("Color Image", cv_image->image);

   char key = cv::waitKey(1);

   if (key == 27 /* ESC */ || key == 'q')
      {
      cv::destroyAllWindows();
      rclcpp::shutdown();
      }

   return;
   }

void Openni2Subscriber::depth_callback(const sensor_msgs::msg::Image::SharedPtr image) const
   {
   RCLCPP_INFO(get_logger(), "Received Depth Image (%d,%d) Encoding %s\n", image->width, image->height,
               image->encoding.c_str());
#if 0
   uint16_t* data = reinterpret_cast<uint16_t*>(&image->data[0]);
   uint16_t max = 0;
   uint16_t min = 65000;
   for (size_t i = 0; i < image->width * image->height; i++)
      {
      //      std::cout << data[i] << std::endl;
      if (data[i] > max)
         {
         max = data[i];
         }

      if ((data[i] > 0) && (data[i] < min))
         {
         min = data[i];
         }
      }

   std::cout << "Max = " << max << " Min = " << min << std::endl;
#endif
   using namespace cv_bridge;

   CvImagePtr image16 = toCvCopy(image);

   cv::Mat img8(image16->image.rows, image16->image.cols, CV_8UC1);

   cv::normalize(image16->image, img8, 0.0, 255.0, cv::NORM_MINMAX, CV_8U);

   cv::imshow("Depth Image", img8);

   char key = cv::waitKey(1);

   if (key == 27 /* ESC */ || key == 'q')
      {
      cv::destroyAllWindows();
      rclcpp::shutdown();
      }

   return;
   }


int main(int argc, char* argv[])
   {
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<Openni2Subscriber>());
   rclcpp::shutdown();

   return 0;
   }
