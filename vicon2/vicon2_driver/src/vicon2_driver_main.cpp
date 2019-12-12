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
#include <iostream>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  ViconDriver vd;

  if(vd.connect_vicon())
  {
    vd.start_vicon();
  }
  vd.stop_vicon();
  rclcpp::shutdown();

  return 0;
}
