cmake_minimum_required(VERSION 3.1.0)

project(SDF_test)
set(CMAKE_BUILD_TYPE "Release")
# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
# View the makefile commands during build
#set(CMAKE_VERBOSE_MAKEFILE on)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()


find_package(OpenCV REQUIRED)
if(NOT OPENCV_FOUND)
    message(FATAL_ERROR "\n\n OpenCV package is missing!\n\n")
endif()

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

set(DEPENDENCIES realsense2)
if(NOT WIN32)
    list(APPEND DEPENDENCIES m ${OpenCV_LIBS})
endif()


add_executable(SDF_test test.cpp src/tsdf/sdf.cpp)

target_link_libraries(SDF_test ${DEPENDENCIES})
