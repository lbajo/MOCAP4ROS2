/*
 *  Copyright (c) 2018, Intelligent Robotics Core S.L. All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Intelligent Robotics Core nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
*/
/* Software License Agreement (BSD License) */
/* Author: David Vargas Frutos - dvargasfr@gmail.com */

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
  void process_markers(const rclcpp::Time& frame_time, unsigned int vicon_frame_num);
  void marker_to_tf(vicon2_msgs::msg::Marker marker, int marker_num, const rclcpp::Time& frame_time);

};
#endif
