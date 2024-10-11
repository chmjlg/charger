##
# Copyright Notice:
# Copyright(c) 2001 - 2024 by JLG Industries.All Rights Reserved.
#
# The information contained herein is confidential and proprietary property
# of JLG Industries, and may be privileged.The dissemination, distribution,
# use, copying, transfer, or disclosure of such information is strictly
# prohibited except by express written agreement with JLG Industries,
# or with one of its subsidiary or parent companies.
#
# Description:
# CMakeLists build file
##

cmake_minimum_required(VERSION 3.8)
project(charger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(charger_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# transform listener
add_executable(charger
  src/charger.cpp
)

ament_target_dependencies(charger
  rclcpp
  std_msgs
  charger_msgs
  tf2_ros
  nav_msgs
  geometry_msgs
)

install(TARGETS
  charger
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
  

ament_package()