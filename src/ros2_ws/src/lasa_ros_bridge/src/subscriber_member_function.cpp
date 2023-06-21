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
#include <geometry_msgs/msg/pose_stamped.hpp>
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("vrpn")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/vrpn/Object_base", 100, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }
ssssss
private:
  void topic_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // Camera position in map frame
        //msg->header;
        double px = msg->pose.position.x;
        double py = msg->pose.position.y;
        double pz = msg->pose.position.z;
        double ox = msg->pose.orientation.x;
        double oy = msg->pose.orientation.y;
        double oz = msg->pose.orientation.z;
        double ow = msg->pose.orientation.w; 
     //Output the measure
       //RCLCPP_INFO(get_logger(), "Received pose in '%s' frame : X: %.2f Y: %.2f Z: %.2f - R: %.2f P: %.2f Y: %.2f - Timestamp: %u.%u sec ",
         //        msg->header.frame_id.c_str(),px, py, pz);
    }
  
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
