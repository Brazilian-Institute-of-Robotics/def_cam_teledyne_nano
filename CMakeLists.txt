cmake_minimum_required(VERSION 2.8.3)

project(def_genie_teledyne)

set(APP_NAME def_teledyne_node)
set(CMAKE_VERBOSE_MAKEFILE OFF)
set(CMAKE_CXX_COMPILER g++)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11 -pthread -w")

find_package(PkgConfig)

find_package(OpenCV 3 REQUIRED)
find_package(Threads  REQUIRED)
find_package(catkin  REQUIRED COMPONENTS roscpp cmake_modules message_generation image_transport cv_bridge)

FILE(GLOB_RECURSE HEADER_FILES "include/*.h*")
FILE(GLOB_RECURSE SRC_FILES "src/camera.cpp" "src/application.cpp")
set(SOURCES /usr/dalsa/GigeV/examples/common/GevUtils.c)

catkin_package(
    INCLUDE_DIRS   include
    LIBRARIES      ${PROJECT_NAME}
    CATKIN_DEPENDS
    DEPENDS        OpenCV Threads
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
include_directories("${PROJECT_SOURCE_DIR}" "${CMAKE_SOURCE_DIR}")

include_directories(
    /usr/dalsa/GigeV/include  
    /usr/dalsa/GigeV/examples/common
    /opt/genicam_v3_0/library/CPP/include
)

link_directories(
    /usr/lib 
    /usr/local/lib 
    /opt/genicam_v3_0/bin/Linux64_x64
    /usr/dalsa/GigeV/lib
)

set(SHARED_LIBRARIES Xext X11 GevApi CorW32 GenApi_gcc421_v3_0 GCBase_gcc421_v3_0 pthread stdc++ )

add_executable(camera_example src/application.cpp ${SOURCES} ${SRC_FILES} ${HEADER_FILES})
target_link_libraries(camera_example pthread ${catkin_LIBRARIES} ${SHARED_LIBRARIES} stdc++ ${OpenCV_LIBS})
