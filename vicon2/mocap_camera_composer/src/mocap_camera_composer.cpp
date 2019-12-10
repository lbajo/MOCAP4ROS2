/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Intelligent Robotics Core S.L.
 *  All rights reserved.
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
 **********************************************************************/

/* Author: David Vargas Frutos - dvargasfr@gmail.com */

#include "mocap_camera_composer.h"

using std::min;
using std::max;
using std::string;
using std::map;
using std::stringstream;
using std::placeholders::_1;

void MocapCameraComposer::start_composer()
{
  marker_sub_ = camera_composer_node->create_subscription<vicon2_msgs::msg::Markers>(
    tracked_frame_suffix_ + "/markers", 100, std::bind(&MocapCameraComposer::marker_to_tf, this, _1));
}

//void MocapCameraComposer::marker_to_tf(vicon2_msgs::msg::Marker marker, int marker_num, const rclcpp::Time& frame_time)
void MocapCameraComposer::marker_to_tf(const vicon2_msgs::msg::Markers::SharedPtr markers_msg)
{
  int marker_id = 0;
  string tracked_frame;
  std::vector<geometry_msgs::msg::TransformStamped> transforms;

  for (vicon2_msgs::msg::Marker marker : markers_msg->markers){
    RCLCPP_INFO(camera_composer_node->get_logger(), "marker name %s", marker.marker_name);
    marker_id++;
    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(marker.translation.x/ 1000, marker.translation.y/ 1000, marker.translation.z/ 1000));
    transform.setRotation(tf2::Quaternion(0,0,0,1));
    tracked_frame = tracked_frame_suffix_ + "/tf_marker_" + std::to_string(marker_id);
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = markers_msg->header.stamp;
    tf_msg.header.frame_id = tf_ref_frame_id_;
    tf_msg.child_frame_id = tracked_frame;
    tf_msg.transform.translation.x = transform.getOrigin().x();
    tf_msg.transform.translation.y = transform.getOrigin().y();
    tf_msg.transform.translation.z = transform.getOrigin().z();
    tf_msg.transform.rotation.x = transform.getRotation().x();
    tf_msg.transform.rotation.y = transform.getRotation().y();
    tf_msg.transform.rotation.z = transform.getRotation().z();
    tf_msg.transform.rotation.w = transform.getRotation().w();
    transforms.push_back(tf_msg);
  }
  tf_broadcaster_->sendTransform(transforms);
}

MocapCameraComposer::MocapCameraComposer()
{
  camera_composer_node = rclcpp::Node::make_shared("camera_composer_node");
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(camera_composer_node);
  tracked_frame_suffix_ = "vicon";
  tf_ref_frame_id_ = "vicon_world";
}
