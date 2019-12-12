// Copyright 2019 Intelligent Robotics Lab
// Author: David Vargas Frutos
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

#ifndef VICON2_DRIVER_HPP_
#define VICON2_DRIVER_HPP_

#include <iostream>
#include <sstream>

#include "rclcpp/time.hpp"

#include "vicon2_msgs/msg/marker.hpp"
#include "vicon2_msgs/msg/markers.hpp"
#include "vicon2_msgs/srv/vicon_calib_seg.hpp"
#include "vicon2_msgs/srv/vicon_grab_pose.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/buffer_core.h"
#include "tf2_ros/transform_broadcaster.h"

#include "rclcpp/node_interfaces/node_logging.hpp"

class MocapCameraComposer
{
public:
  std::string tracked_frame_suffix_;
  std::string tf_ref_frame_id_;
  MocapCameraComposer();
  void start_composer();

private:
  rclcpp::Node::SharedPtr camera_composer_node;
  rclcpp::Subscription<vicon2_msgs::msg::Markers>::SharedPtr marker_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  //void marker_to_tf(vicon2_msgs::msg::Marker marker, int marker_num, const rclcpp::Time& frame_time);
  void marker_to_tf(const vicon2_msgs::msg::Markers::SharedPtr markers_msg);
};
#endif
