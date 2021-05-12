cmake_minimum_required(VERSION 3.1.3)

project(def_cam_teledyne_nano)

set(CMAKE_CXX_STANDARD 11)

find_package(gige_vision_framework_dalsa REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  sensor_msgs
  image_transport
  camera_info_manager
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${gige_vision_framework_dalsa_INCLUDE_DIRS}
)

add_executable(camera_example
  src/main.cpp
  src/CameraPublisher.cpp
  src/CameraAPI.cpp
)

target_link_libraries(camera_example
  ${gige_vision_framework_dalsa_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

install(TARGETS camera_example RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)

  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)

endif()
