cmake_minimum_required(VERSION 3.8)
project(orb_slam3_ros2)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)


add_definitions(-Wall -O3 -march=native)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Include directories
include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../../../
  ${PROJECT_SOURCE_DIR}/../../../include
  ${PROJECT_SOURCE_DIR}/../../../Thirdparty/Sophus
  ${PROJECT_SOURCE_DIR}/../../../include/CameraModels
  ${PROJECT_SOURCE_DIR}/../../../lib
)

# Libraries
set(LIBS 
  ${OpenCV_LIBRARIES} 
  ${EIGEN3_LIBRARIES}
  ${Pangolin_LIBRARIES}
  Boost::system
  ${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
  ${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM3.so
)

# Executables
# add_executable(mono src/ros_mono.cc)
# ament_target_dependencies(mono rclcpp sensor_msgs image_transport cv_bridge)
# target_link_libraries(mono ${LIBS})


add_executable(mono_inertial src/ros_mono_inertial.cc)
ament_target_dependencies(mono_inertial rclcpp sensor_msgs geometry_msgs image_transport cv_bridge)
target_link_libraries(mono_inertial ${LIBS})

add_executable(mono src/ros_mono.cc)
ament_target_dependencies(mono rclcpp sensor_msgs geometry_msgs image_transport cv_bridge)
target_link_libraries(mono ${LIBS})


set_target_properties(mono_inertial PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH_USE_LINK_PATH TRUE)

set_target_properties(mono PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH_USE_LINK_PATH TRUE)

# Install targets
install(TARGETS mono_inertial mono
  DESTINATION lib/${PROJECT_NAME})

ament_package()
