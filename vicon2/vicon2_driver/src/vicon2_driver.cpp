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

#include "vicon2_driver.h"

using std::min;
using std::max;
using std::string;
using std::map;
using std::stringstream;

using namespace ViconDataStreamSDK::CPP;

string Enum2String(const Direction::Enum i_Direction)
{
  switch (i_Direction)
  {
    case Direction::Forward:
      return "Forward";
    case Direction::Backward:
      return "Backward";
    case Direction::Left:
      return "Left";
    case Direction::Right:
      return "Right";
    case Direction::Up:
      return "Up";
    case Direction::Down:
      return "Down";
    default:
      return "Unknown";
  }
}

string Enum2String(const Result::Enum i_result)
{
  switch (i_result)
  {
    case Result::ClientAlreadyConnected:
      return "ClientAlreadyConnected";
    case Result::ClientConnectionFailed:
      return "";
    case Result::CoLinearAxes:
      return "CoLinearAxes";
    case Result::InvalidDeviceName:
      return "InvalidDeviceName";
    case Result::InvalidDeviceOutputName:
      return "InvalidDeviceOutputName";
    case Result::InvalidHostName:
      return "InvalidHostName";
    case Result::InvalidIndex:
      return "InvalidIndex";
    case Result::InvalidLatencySampleName:
      return "InvalidLatencySampleName";
    case Result::InvalidMarkerName:
      return "InvalidMarkerName";
    case Result::InvalidMulticastIP:
      return "InvalidMulticastIP";
    case Result::InvalidSegmentName:
      return "InvalidSegmentName";
    case Result::InvalidSubjectName:
      return "InvalidSubjectName";
    case Result::LeftHandedAxes:
      return "LeftHandedAxes";
    case Result::NoFrame:
      return "NoFrame";
    case Result::NotConnected:
      return "NotConnected";
    case Result::NotImplemented:
      return "NotImplemented";
    case Result::ServerAlreadyTransmittingMulticast:
      return "ServerAlreadyTransmittingMulticast";
    case Result::ServerNotTransmittingMulticast:
      return "ServerNotTransmittingMulticast";
    case Result::Success:
      return "Success";
    case Result::Unknown:
      return "Unknown";
    default:
      return "unknown";
  }
}

void ViconDriver::set_settings_vicon()
{
  Result::Enum result(Result::Unknown);
  if (stream_mode_ == "ServerPush")
  {
    result = client.SetStreamMode(StreamMode::ServerPush).Result;
  }
  else if (stream_mode_ == "ClientPull")
  {
    result = client.SetStreamMode(StreamMode::ClientPull).Result;
  }
  else
  {
    RCLCPP_FATAL(vicon_node->get_logger(), "Unknown stream mode -- options are ServerPush, ClientPull");
    rclcpp::shutdown();
  }

  RCLCPP_INFO(vicon_node->get_logger(), "Setting Stream Mode to %s : %s", stream_mode_.c_str(), Enum2String(result).c_str());

  client.SetAxisMapping(Direction::Forward, Direction::Left, Direction::Up); // 'Z-up'
  Output_GetAxisMapping _Output_GetAxisMapping = client.GetAxisMapping();

  RCLCPP_INFO(vicon_node->get_logger(), "Axis Mapping: X-%s Y-%s Z-%s",
    Enum2String(_Output_GetAxisMapping.XAxis).c_str(),
    Enum2String(_Output_GetAxisMapping.YAxis).c_str(),
    Enum2String(_Output_GetAxisMapping.ZAxis).c_str());

  client.EnableSegmentData();
  RCLCPP_INFO(vicon_node->get_logger(), "IsSegmentDataEnabled? %s",client.IsSegmentDataEnabled().Enabled?"true":"false");

  Output_GetVersion _Output_GetVersion = client.GetVersion();
  RCLCPP_INFO(vicon_node->get_logger(), "Version: %d.%d.%d",
    _Output_GetVersion.Major,
    _Output_GetVersion.Minor,
    _Output_GetVersion.Point
  );

  if(publish_markers_)
  {
    marker_pub_ = vicon_node->create_publisher<vicon2_msgs::msg::Markers>(tracked_frame_suffix_ + "/markers", 100);
  }
}

void ViconDriver::start_vicon()
{
  set_settings_vicon();
  rclcpp::WallRate d(1.0 / 240.0);
  while (rclcpp::ok())
  {
    while (client.GetFrame().Result != Result::Success && rclcpp::ok())
    {
      // RCLCPP_INFO(vicon_node->get_logger(), "getFrame returned false");
      d.sleep();
    }
    now_time = vicon_node->now();
    process_frame();
  }
}

bool ViconDriver::stop_vicon()
{
  RCLCPP_INFO(vicon_node->get_logger(), "Disconnecting from Vicon DataStream SDK");
  client.Disconnect();
  RCLCPP_INFO(vicon_node->get_logger(), "... disconnected");
  return true;
}

void ViconDriver::process_frame()
{
  static rclcpp::Time lastTime;
  Output_GetFrameNumber OutputFrameNum = client.GetFrameNumber();

  int frameDiff = 0;
  if (lastFrameNumber != 0)
  {
    frameDiff = OutputFrameNum.FrameNumber - lastFrameNumber;
    frameCount += frameDiff;
    if ((frameDiff) > 1)
    {
      droppedFrameCount += frameDiff;
      double droppedFramePct = (double)droppedFrameCount / frameCount * 100;
      RCLCPP_DEBUG(vicon_node->get_logger(), "%d more (total %d / %d, %f %%) frame(s) dropped. Consider adjusting rates",
          frameDiff, droppedFrameCount, frameCount, droppedFramePct);
    }
  }
  lastFrameNumber = OutputFrameNum.FrameNumber;

  if (frameDiff != 0)
  {
    rclcpp::Duration vicon_latency(client.GetLatencyTotal().Total);
    if(publish_markers_)
    {
      process_markers(now_time - vicon_latency, lastFrameNumber);
    }
    lastTime = now_time;
  }
}

void ViconDriver::process_markers(const rclcpp::Time& frame_time, unsigned int vicon_frame_num)
{
  int marker_cnt = 0;
  if (not marker_data_enabled)
  {
    marker_data_enabled = true;
    client.EnableMarkerData();
    RCLCPP_INFO(vicon_node->get_logger(), "IsMarkerDataEnabled? %s",client.IsMarkerDataEnabled().Enabled?"true":"false");
  }
  if (not unlabeled_marker_data_enabled)
  {
    unlabeled_marker_data_enabled = true;
    client.EnableUnlabeledMarkerData();
    RCLCPP_INFO(vicon_node->get_logger(), "IsUnlabeledMarkerDataEnabled? %s",client.IsUnlabeledMarkerDataEnabled().Enabled?"true":"false");
  }
  n_markers = 0;
  vicon2_msgs::msg::Markers markers_msg;
  markers_msg.header.stamp = frame_time;
  markers_msg.frame_number = vicon_frame_num;
  unsigned int UnlabeledMarkerCount = client.GetUnlabeledMarkerCount().MarkerCount;
  RCLCPP_INFO(vicon_node->get_logger(), "# unlabeled markers: %d", UnlabeledMarkerCount);
  n_markers += UnlabeledMarkerCount;
  n_unlabeled_markers = UnlabeledMarkerCount;
  for (unsigned int UnlabeledMarkerIndex = 0; UnlabeledMarkerIndex < UnlabeledMarkerCount; ++UnlabeledMarkerIndex)
  {
    // Get the global marker translationSegmentPublisher
    Output_GetUnlabeledMarkerGlobalTranslation _Output_GetUnlabeledMarkerGlobalTranslation =
        client.GetUnlabeledMarkerGlobalTranslation(UnlabeledMarkerIndex);

    if (_Output_GetUnlabeledMarkerGlobalTranslation.Result == Result::Success)
    {
      vicon2_msgs::msg::Marker this_marker;
      this_marker.translation.x = _Output_GetUnlabeledMarkerGlobalTranslation.Translation[0];
      this_marker.translation.y = _Output_GetUnlabeledMarkerGlobalTranslation.Translation[1];
      this_marker.translation.z = _Output_GetUnlabeledMarkerGlobalTranslation.Translation[2];
      this_marker.occluded = false;
      markers_msg.markers.push_back(this_marker);

      marker_to_tf(this_marker, marker_cnt, frame_time);
      marker_cnt++;
    }
    else
    {
      RCLCPP_WARN(vicon_node->get_logger(), "GetUnlabeledMarkerGlobalTranslation failed (result = %s)",
          Enum2String(_Output_GetUnlabeledMarkerGlobalTranslation.Result).c_str());
    }
  }
  marker_pub_->publish(markers_msg);
}

void ViconDriver::marker_to_tf(vicon2_msgs::msg::Marker marker, int marker_num, const rclcpp::Time& frame_time)
{
  tf2::Transform transform;
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  string tracked_frame;
  geometry_msgs::msg::TransformStamped tf_msg;

  transform.setOrigin(tf2::Vector3(marker.translation.x/ 1000, marker.translation.y/ 1000, marker.translation.z/ 1000));
  transform.setRotation(tf2::Quaternion(0,0,0,1));
  stringstream marker_num_str;
  marker_num_str << marker_num;
  tracked_frame = tracked_frame_suffix_ + "/marker_tf_" + marker_num_str.str();

  tf_msg.header.stamp = frame_time;
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
  tf_broadcaster_->sendTransform(transforms);
}

ViconDriver::ViconDriver()
{
  vicon_node = rclcpp::Node::make_shared("vicon_node");
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(vicon_node);
  stream_mode_ = "ClientPull";
  host_name_ = "192.168.10.1:801";
  tf_ref_frame_id_ = "vicon_world";
  tracked_frame_suffix_ = "vicon";
  publish_markers_ = true;
  marker_data_enabled = false;
  unlabeled_marker_data_enabled = false;
  lastFrameNumber = 0;
  frameCount = 0;
  droppedFrameCount = 0;
  n_markers = 0;
  n_unlabeled_markers = 0;
}

bool ViconDriver::connect_vicon()
{
  RCLCPP_WARN(vicon_node->get_logger(), "Trying to connect to Vicon DataStream SDK at %s ...", host_name_.c_str());

  if(client.Connect(host_name_).Result == Result::Success)
  {
    RCLCPP_INFO(vicon_node->get_logger(), "... connected!");
  }
  else
  {
    RCLCPP_INFO(vicon_node->get_logger(), "... not connected :(");
  }

  return client.IsConnected().Connected;
}
