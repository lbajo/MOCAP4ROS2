cmake_minimum_required(VERSION 3.5)

project(vicon2_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(mocap4ros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Threads REQUIRED)
# find_package(Boost REQUIRED COMPONENTS thread)

set(CMAKE_USE_WIN32_THREADS_INIT 1)

find_path(DATASTREAMCLIENT_INCLUDE_DIR DataStreamclient.h)
find_library(DATASTREAMCLIENT_LIBRARY ViconDataStreamSDK_CPP)
if(DATASTREAMCLIENT_INCLUDE_DIR AND DATASTREAMCLIENT_LIBRARY)
  set(DATASTREAMCLIENT_FOUND 1)
  message(STATUS "#### DATASTREAMCLIENT_INCLUDE_DIR: " ${DATASTREAMCLIENT_INCLUDE_DIR})
  message(STATUS "#### DATASTREAMCLIENT_LIBRARY: " ${DATASTREAMCLIENT_LIBRARY})
endif()
        
set(dependencies
  ament_index_cpp
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_ros
  mocap4ros_msgs
)

include_directories(
  include
  ${DATASTREAMCLIENT_INCLUDE_DIR}
)

add_library(${PROJECT_NAME} SHARED src/vicon2_driver.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(vicon2_driver_main
  src/vicon2_driver_main.cpp
)
ament_target_dependencies(vicon2_driver_main ${dependencies})
target_link_libraries(vicon2_driver_main
  ${PROJECT_NAME}
  ${DATASTREAMCLIENT_LIBRARY}
  # ${CMAKE_THREAD_LIBS_INIT}
)

ament_target_dependencies(vicon2_driver ${dependencies})
target_link_libraries(vicon2_driver
  ${DATASTREAMCLIENT_LIBRARY}
  # ${CMAKE_THREAD_LIBS_INIT}
  # Boost::system
  # Boost::thread
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  vicon2_driver_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include ${DATASTREAMCLIENT_INCLUDE_DIR})
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
