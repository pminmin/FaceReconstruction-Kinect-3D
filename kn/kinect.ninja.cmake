# kinect.ninja

# Find Kinect SDK 2.0 & Set `KinectRoot`
if(DEFINED ENV{KINECTSDK20_DIR})
  set(KinectRoot $ENV{KINECTSDK20_DIR})
  message(STATUS "Please add ${KinectRoot}/bin to %PATH%")
else()
  message(FATAL_ERROR "No Kinect SDK 2.0")
endif()

# Add Kinect Library
include_directories(${KinectRoot}/inc)
link_directories(${KinectRoot}/Lib/x86)
set(KinectLibs Kinect20 Kinect20.Face Kinect20.Fusion)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Project Configuration
include_directories(${CMAKE_CURRENT_LIST_DIR}/include)
file(GLOB SRC ${CMAKE_CURRENT_LIST_DIR}/include/*.h*
              ${CMAKE_CURRENT_LIST_DIR}/src/*.c*)

add_library(kinect.ninja STATIC ${SRC})
target_link_libraries(kinect.ninja ${KinectLibs} ${OpenCV_LIBS})

# What's More
