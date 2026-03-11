cmake_minimum_required(VERSION 3.0.2)
project(loop_relocator)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
SET(CMAKE_BUILD_TYPE "Release")
#SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
#SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  carstatemsgs
  pcl_conversions
  message_filters
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf_conversions
  tf2_ros
  Aruco_detection
)
find_package (GeographicLib REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_package(Eigen3 REQUIRED)
# find_package(Ceres REQUIRED)
# find_package(GTSAM REQUIRED QUIET)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")
catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(utils_lib src/utils.cpp)


# add_executable(btc_place_recognition example/place_recognition.cpp src/btc.cpp include/ikd-Tree/ikd_Tree.cpp)
# target_link_libraries(btc_place_recognition 
#         ${catkin_LIBRARIES} 
#         ${PCL_LIBRARIES} 
#         ${OpenCV_LIBS} 
#         ${CERES_LIBRARIES} 
#         tbb
#         utils_lib)
add_executable(place_recognition_node src/place_recognition_node.cpp include/ikd-Tree/ikd_Tree.cpp src/submap_match.cpp src/RTK/rtk_relocator.cpp src/QR_code/qr_relocator.cpp)
target_link_libraries(place_recognition_node 
        ${catkin_LIBRARIES} 
        ${PCL_LIBRARIES} 
        ${OpenCV_LIBS} 
        ${CERES_LIBRARIES} 
        ${GeographicLib_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        # tbb
        utils_lib
        )

# ── QR Target Visualizer ──────────────────────────────────────────────────────
add_executable(qr_target_visualizer
        src/QR_code/qr_target_visualizer.cpp)
target_link_libraries(qr_target_visualizer
        ${catkin_LIBRARIES}
        ${YAML_CPP_LIBRARIES})
              
      
