// Copyright 2019 Intelligent Robotics Lab
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
//
// Author: David Vargas Frutos <david.vargas@urjc.es>

#ifndef VICON2_DRIVER_HPP_
#define VICON2_DRIVER_HPP_

#include <iostream>
#include <sstream>
#include <string>
#include <memory>

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

#include "Client.h"

class ViconDriver
{
public:
  std::string tf_ref_frame_id_;
  std::string tracked_frame_suffix_;

  ViconDriver();
  bool connect_vicon();
  void start_vicon();
  bool stop_vicon();

private:
  ViconDataStreamSDK::CPP::Client client;
  rclcpp::Node::SharedPtr vicon_node;
  rclcpp::Time now_time;
  rclcpp::Publisher<vicon2_msgs::msg::Markers>::SharedPtr marker_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string stream_mode_;
  std::string host_name_;
  bool publish_markers_;
  bool marker_data_enabled;
  bool unlabeled_marker_data_enabled;
  unsigned int lastFrameNumber;
  unsigned int frameCount;
  unsigned int droppedFrameCount;
  unsigned int n_markers;
  unsigned int n_unlabeled_markers;

  void set_settings_vicon();
  void process_frame();
  void process_markers(const rclcpp::Time & frame_time, unsigned int vicon_frame_num);
  void marker_to_tf(
    vicon2_msgs::msg::Marker marker,
    int marker_num, const rclcpp::Time & frame_time);
};
#endif  // VICON2_DRIVER_HPP_
