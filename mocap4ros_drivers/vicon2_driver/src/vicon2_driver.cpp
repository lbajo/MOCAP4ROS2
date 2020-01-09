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

#include <string>
#include <vector>
#include <memory>
#include "vicon2_driver/vicon2_driver.hpp"
#include "lifecycle_msgs/msg/state.hpp"

using std::min;
using std::max;
using std::string;
using std::map;
using std::stringstream;

string Enum2String(const ViconDataStreamSDK::CPP::Direction::Enum i_Direction)
{
  switch (i_Direction) {
    case ViconDataStreamSDK::CPP::Direction::Forward:
      return "Forward";
    case ViconDataStreamSDK::CPP::Direction::Backward:
      return "Backward";
    case ViconDataStreamSDK::CPP::Direction::Left:
      return "Left";
    case ViconDataStreamSDK::CPP::Direction::Right:
      return "Right";
    case ViconDataStreamSDK::CPP::Direction::Up:
      return "Up";
    case ViconDataStreamSDK::CPP::Direction::Down:
      return "Down";
    default:
      return "Unknown";
  }
}

string Enum2String(const ViconDataStreamSDK::CPP::Result::Enum i_result)
{
  switch (i_result) {
    case ViconDataStreamSDK::CPP::Result::ClientAlreadyConnected:
      return "ClientAlreadyConnected";
    case ViconDataStreamSDK::CPP::Result::ClientConnectionFailed:
      return "";
    case ViconDataStreamSDK::CPP::Result::CoLinearAxes:
      return "CoLinearAxes";
    case ViconDataStreamSDK::CPP::Result::InvalidDeviceName:
      return "InvalidDeviceName";
    case ViconDataStreamSDK::CPP::Result::InvalidDeviceOutputName:
      return "InvalidDeviceOutputName";
    case ViconDataStreamSDK::CPP::Result::InvalidHostName:
      return "InvalidHostName";
    case ViconDataStreamSDK::CPP::Result::InvalidIndex:
      return "InvalidIndex";
    case ViconDataStreamSDK::CPP::Result::InvalidLatencySampleName:
      return "InvalidLatencySampleName";
    case ViconDataStreamSDK::CPP::Result::InvalidMarkerName:
      return "InvalidMarkerName";
    case ViconDataStreamSDK::CPP::Result::InvalidMulticastIP:
      return "InvalidMulticastIP";
    case ViconDataStreamSDK::CPP::Result::InvalidSegmentName:
      return "InvalidSegmentName";
    case ViconDataStreamSDK::CPP::Result::InvalidSubjectName:
      return "InvalidSubjectName";
    case ViconDataStreamSDK::CPP::Result::LeftHandedAxes:
      return "LeftHandedAxes";
    case ViconDataStreamSDK::CPP::Result::NoFrame:
      return "NoFrame";
    case ViconDataStreamSDK::CPP::Result::NotConnected:
      return "NotConnected";
    case ViconDataStreamSDK::CPP::Result::NotImplemented:
      return "NotImplemented";
    case ViconDataStreamSDK::CPP::Result::ServerAlreadyTransmittingMulticast:
      return "ServerAlreadyTransmittingMulticast";
    case ViconDataStreamSDK::CPP::Result::ServerNotTransmittingMulticast:
      return "ServerNotTransmittingMulticast";
    case ViconDataStreamSDK::CPP::Result::Success:
      return "Success";
    case ViconDataStreamSDK::CPP::Result::Unknown:
      return "Unknown";
    default:
      return "unknown";
  }
}

void ViconDriver::set_settings_vicon()
{
  ViconDataStreamSDK::CPP::Result::Enum result(ViconDataStreamSDK::CPP::Result::Unknown);
  if (stream_mode_ == "ServerPush") {
    result = client.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ServerPush).Result;
  } else if (stream_mode_ == "ClientPull") {
    result = client.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ClientPull).Result;
  } else {
    RCLCPP_FATAL(get_logger(), "Unknown stream mode -- options are ServerPush, ClientPull");
    rclcpp::shutdown();
  }

  RCLCPP_INFO(get_logger(), "Setting Stream Mode to %s : %s",
    stream_mode_.c_str(), Enum2String(result).c_str());

  client.SetAxisMapping(ViconDataStreamSDK::CPP::Direction::Forward,
    ViconDataStreamSDK::CPP::Direction::Left, ViconDataStreamSDK::CPP::Direction::Up);
  ViconDataStreamSDK::CPP::Output_GetAxisMapping _Output_GetAxisMapping = client.GetAxisMapping();

  RCLCPP_INFO(get_logger(), "Axis Mapping: X-%s Y-%s Z-%s",
    Enum2String(_Output_GetAxisMapping.XAxis).c_str(),
    Enum2String(_Output_GetAxisMapping.YAxis).c_str(),
    Enum2String(_Output_GetAxisMapping.ZAxis).c_str());

  client.EnableSegmentData();

  RCLCPP_INFO(get_logger(),
    "IsSegmentDataEnabled? %s",
    client.IsSegmentDataEnabled().Enabled ? "true" : "false");

  ViconDataStreamSDK::CPP::Output_GetVersion _Output_GetVersion = client.GetVersion();

  RCLCPP_INFO(get_logger(), "Version: %d.%d.%d",
    _Output_GetVersion.Major,
    _Output_GetVersion.Minor,
    _Output_GetVersion.Point
  );

  if (publish_markers_) {
    marker_pub_ = create_publisher<mocap4ros_msgs::msg::Markers>(
      tracked_frame_suffix_ + "/markers", 100);
  }
}

void ViconDriver::start_vicon()
{
  set_settings_vicon();
  rclcpp::WallRate d(1.0 / 240.0);
  while (rclcpp::ok()) {
    while (client.GetFrame().Result != ViconDataStreamSDK::CPP::Result::Success && rclcpp::ok()) {
      RCLCPP_INFO(get_logger(), "getFrame returned false");
      d.sleep();
    }
    now_time = this->now();
    process_frame();
  }
}

bool ViconDriver::stop_vicon()
{
  RCLCPP_INFO(get_logger(), "Disconnecting from Vicon DataStream SDK");
  client.Disconnect();
  RCLCPP_INFO(get_logger(), "... disconnected");
  return true;
}

void ViconDriver::process_frame()
{
  static rclcpp::Time lastTime;
  ViconDataStreamSDK::CPP::Output_GetFrameNumber OutputFrameNum = client.GetFrameNumber();

  int frameDiff = 0;
  if (lastFrameNumber != 0) {
    frameDiff = OutputFrameNum.FrameNumber - lastFrameNumber;
    frameCount += frameDiff;
    if ((frameDiff) > 1) {
      droppedFrameCount += frameDiff;
      double droppedFramePct = static_cast<double>(droppedFrameCount / frameCount * 100);

      RCLCPP_DEBUG(get_logger(),
        "%d more (total %d / %d, %f %%) frame(s) dropped. Consider adjusting rates",
        frameDiff, droppedFrameCount, frameCount, droppedFramePct);
    }
  }
  lastFrameNumber = OutputFrameNum.FrameNumber;

  if (frameDiff != 0) {
    rclcpp::Duration vicon_latency(client.GetLatencyTotal().Total);
    if (publish_markers_) {
      process_markers(now_time - vicon_latency, lastFrameNumber);
    }
    lastTime = now_time;
  }
}

void ViconDriver::process_markers(const rclcpp::Time & frame_time, unsigned int vicon_frame_num)
{
  int marker_cnt = 0;
  if (!marker_data_enabled) {
    marker_data_enabled = true;
    client.EnableMarkerData();

    RCLCPP_INFO(get_logger(),
      "IsMarkerDataEnabled? %s",
      client.IsMarkerDataEnabled().Enabled ? "true" : "false");
  }
  if (!unlabeled_marker_data_enabled) {
    unlabeled_marker_data_enabled = true;
    client.EnableUnlabeledMarkerData();

    RCLCPP_INFO(get_logger(),
      "IsUnlabeledMarkerDataEnabled? %s",
      client.IsUnlabeledMarkerDataEnabled().Enabled ? "true" : "false");
  }
  n_markers = 0;
  mocap4ros_msgs::msg::Markers markers_msg;
  markers_msg.header.stamp = frame_time;
  markers_msg.frame_number = vicon_frame_num;
  unsigned int UnlabeledMarkerCount = client.GetUnlabeledMarkerCount().MarkerCount;

  RCLCPP_INFO(get_logger(),
    "# unlabeled markers: %d", UnlabeledMarkerCount);
  n_markers += UnlabeledMarkerCount;
  n_unlabeled_markers = UnlabeledMarkerCount;
  for (unsigned int UnlabeledMarkerIndex = 0;
    UnlabeledMarkerIndex < UnlabeledMarkerCount;
    ++UnlabeledMarkerIndex)
  {
    // Get the global marker translationSegmentPublisher
    ViconDataStreamSDK::CPP::Output_GetUnlabeledMarkerGlobalTranslation
      _Output_GetUnlabeledMarkerGlobalTranslation =
      client.GetUnlabeledMarkerGlobalTranslation(UnlabeledMarkerIndex);

    if (_Output_GetUnlabeledMarkerGlobalTranslation.Result ==
      ViconDataStreamSDK::CPP::Result::Success)
    {
      mocap4ros_msgs::msg::Marker this_marker;
      this_marker.translation.x = _Output_GetUnlabeledMarkerGlobalTranslation.Translation[0];
      this_marker.translation.y = _Output_GetUnlabeledMarkerGlobalTranslation.Translation[1];
      this_marker.translation.z = _Output_GetUnlabeledMarkerGlobalTranslation.Translation[2];
      this_marker.occluded = false;
      markers_msg.markers.push_back(this_marker);

      marker_to_tf(this_marker, marker_cnt, frame_time);
      marker_cnt++;
    } else {
      RCLCPP_WARN(get_logger(),
        "GetUnlabeledMarkerGlobalTranslation failed (result = %s)",
        Enum2String(_Output_GetUnlabeledMarkerGlobalTranslation.Result).c_str());
    }
  }
  marker_pub_->publish(markers_msg);
}

void ViconDriver::marker_to_tf(
  mocap4ros_msgs::msg::Marker marker,
  int marker_num, const rclcpp::Time & frame_time)
{
  tf2::Transform transform;
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  string tracked_frame;
  geometry_msgs::msg::TransformStamped tf_msg;

  transform.setOrigin(tf2::Vector3(marker.translation.x / 1000,
    marker.translation.y / 1000,
    marker.translation.z / 1000));
  transform.setRotation(tf2::Quaternion(0, 0, 0, 1));
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

ViconDriver::ViconDriver(const rclcpp::NodeOptions node_options)
: rclcpp_lifecycle::LifecycleNode("vicon2_driver_node", node_options)
{
  // vicon_node = rclcpp::Node::make_shared("vicon_node");
  // parameters_client = std::make_shared<rclcpp::SyncParametersClient>(vicon_node);
  // tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(vicon_node);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
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

  /*
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
  auto future_result = client_change_state_->async_send_request(request);
  */

  /*
  declare_parameter("test_param", "");
  get_parameter("test_param", params_file_);
  RCLCPP_INFO(get_logger(), "test_param [%s]", params_file_);
  */

  /*
  declare_parameter("params_file", "");
  get_parameter("params_file", params_file_);
  RCLCPP_INFO(get_logger(), "params_file [%s]", params_file_);
  */

  client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
    "/vicon2_driver/change_state");
  update_pub_ = create_publisher<std_msgs::msg::Empty>(
    "/vicon2_driver/update_notify", rclcpp::QoS(100));

  initParameters();
}

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
ViconDriver::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "State [%s]", state.label());
  RCLCPP_INFO(get_logger(), "[%s] Configuring...", get_name());
  /*...*/
  RCLCPP_INFO(get_logger(), "[%s] Configured", get_name());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ViconDriver::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "State [%s]", state.label());
  RCLCPP_INFO(get_logger(), "[%s] Activating...", get_name());
  update_pub_->on_activate();
  RCLCPP_INFO(get_logger(), "[%s] Activated", get_name());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ViconDriver::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "State [%s]", state.label());
  RCLCPP_INFO(get_logger(), "[%s] Deactivating...", get_name());
  update_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "[%s] Deactivated", get_name());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ViconDriver::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "State [%s]", state.label());
  RCLCPP_INFO(get_logger(), "[%s] Cleaning up...", get_name());
  RCLCPP_INFO(get_logger(), "[%s] Cleaned up", get_name());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ViconDriver::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "State [%s]", state.label());
  RCLCPP_INFO(get_logger(), "[%s] Shutting down...", get_name());
  RCLCPP_INFO(get_logger(), "[%s] Shutted down", get_name());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ViconDriver::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "State [%s]", state.label());
  RCLCPP_ERROR(get_logger(), "[%s] Error transition", get_name());

  return CallbackReturnT::SUCCESS;
}

bool ViconDriver::connect_vicon()
{
  RCLCPP_WARN(get_logger(),
    "Trying to connect to Vicon DataStream SDK at %s ...",
    host_name_.c_str());

  if (client.Connect(host_name_).Result == ViconDataStreamSDK::CPP::Result::Success) {
    RCLCPP_INFO(get_logger(), "... connected!");
  } else {
    RCLCPP_INFO(get_logger(), "... not connected :(");
  }

  return client.IsConnected().Connected;
}

void ViconDriver::initParameters()
{
  std::string paramName = "test_param";
  declare_parameter<std::string>(paramName, "...");

  if (get_parameter_or<std::string>(paramName, myParam, "...")) {
    RCLCPP_WARN(get_logger(),
      "The parameter '%s' is available, using YAML value: %s",
      paramName.c_str(), myParam.c_str());
  } else {
    RCLCPP_WARN(get_logger(),
      "The parameter '%s' is not available, using the default value: %s",
      paramName.c_str(), myParam.c_str());
  }
}
