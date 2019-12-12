cmake_minimum_required(VERSION 3.5)

project(mocap_camera_composer)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vicon2_msgs REQUIRED)
#find_package(mocap_camera_composer REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(rosidl_default_generators REQUIRED)


include_directories(include
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/mocap_camera_composer.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "std_msgs"
)

#add_executable(mocap_camera_composer_main
#  src/mocap_camera_composer_main.cpp
#)

#target_link_libraries(mocap_camera_composer_main
#  ${PROJECT_NAME}
#)

#ament_target_dependencies(mocap_camera_composer_main
#  "rclcpp"
#  "std_msgs"
#)

target_link_libraries(mocap_camera_composer
)
ament_target_dependencies(mocap_camera_composer
  "rclcpp"
  "vicon2_msgs"
  "tf2"
  "tf2_ros"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#install(TARGETS
#  mocap_camera_composer_main
#  DESTINATION lib/${PROJECT_NAME}
#)

#install(DIRECTORY
#  launch
#  DESTINATION share/${PROJECT_NAME}
#)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(vicon2_msgs)
ament_package()
