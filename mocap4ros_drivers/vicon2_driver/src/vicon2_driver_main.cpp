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

#include <iostream>
#include "vicon2_driver/vicon2_driver.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  /*
  ViconDriver vd;
  vd.set_settings_vicon();
  if (vd.connect_vicon()) {
    vd.start_vicon();
  }
  vd.stop_vicon();
  */
  /**/
  auto node = std::make_shared<ViconDriver>();
  rclcpp_lifecycle::State state;
  node->on_configure(state);
  //rclcpp::spin(node->get_node_base_interface());
  /**/
  rclcpp::shutdown();

  return 0;
}

/*
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ViconDriver>();
  //node->connect_vicon();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
*/