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

#ifndef VICON2_DRIVER__VICON2_DRIVER_HPP_
#define VICON2_DRIVER__VICON2_DRIVER_HPP_

#include <iostream>
#include <sstream>
#include <string>
#include <memory>
#include <chrono>


#include "rclcpp/time.hpp"

#include "mocap4ros_msgs/msg/marker.hpp"
#include "mocap4ros_msgs/msg/markers.hpp"
#include "mocap4ros_msgs/srv/vicon_calib_seg.hpp"
#include "mocap4ros_msgs/srv/vicon_grab_pose.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/node_interfaces/node_logging.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include "tf2/buffer_core.h"
#include "tf2_ros/transform_broadcaster.h"


#include "rclcpp/node_interfaces/node_logging.hpp"

#include "ViconDataStreamSDK/DataStreamClient.h"

class ViconDriver : public rclcpp_lifecycle::LifecycleNode
{
public:

  std::string tf_ref_frame_id_;
  std::string tracked_frame_suffix_;

  ViconDriver();

  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);
  bool connect_vicon();
  void set_settings_vicon();
  void start_vicon();
  bool stop_vicon();

private:
  ViconDataStreamSDK::CPP::Client client;
  //rclcpp::Node::SharedPtr vicon_node;
  //std::shared_ptr<rclcpp::SyncParametersClient> parameters_client;
  rclcpp::Time now_time;
  rclcpp::Publisher<mocap4ros_msgs::msg::Markers>::SharedPtr marker_pub_;
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

  std::string params_file_;
  rclcpp::Parameter my_parameter_;


  void process_frame();
  void process_markers(const rclcpp::Time & frame_time, unsigned int vicon_frame_num);
  void marker_to_tf(
    mocap4ros_msgs::msg::Marker marker,
    int marker_num, const rclcpp::Time & frame_time);
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Empty>::SharedPtr update_pub_;
};
#endif  // VICON2_DRIVER__VICON2_DRIVER_HPP_
