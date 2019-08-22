cmake_minimum_required(VERSION 2.8.3)

project(def_cam_teledyne_nano)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11 -w")

find_package(PkgConfig)
find_package(OpenCV 3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslaunch
  cv_bridge
  sensor_msgs
  image_transport
  camera_info_manager
)

roslaunch_add_file_check(launch)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(camera_example
  src/main.cpp
  src/CameraPublisher.cpp
  src/CameraAPI.cpp
)

target_link_libraries(camera_example
  GevApi
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

install(TARGETS camera_example RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
