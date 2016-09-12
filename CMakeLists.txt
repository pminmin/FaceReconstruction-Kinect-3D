project(FaceReconstruction)
cmake_minimum_required(VERSION 2.8)

# OpenCV
find_package(OpenCV REQUIRED)

# OpenGL
find_package(OpenGL REQUIRED)

# SFML
set(SFML_ROOT SFML-2.3.2)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake/Modules")
find_package(SFML COMPONENTS graphics window system)
include_directories(${SFML_INCLUDE_DIR})

# Kinect.Ninja
include(kn/kinect.ninja.cmake)

include_directories(.)
file(GLOB HEADERS *.h*)
file(GLOB SOURCES *.c*)
set(ALL_FILES ${HEADERS} ${SOURCES})

add_executable(${PROJECT_NAME} ${ALL_FILES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${SFML_LIBRARIES}
                      ${OPENGL_glu_LIBRARY} ${OPENGL_gl_LIBRARY} kinect.ninja)