cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)

# Old, working version. Doesn't use separate nodes.
rosbuild_add_boost_directories()
#rosbuild_add_executable(old_keyboard_drive src/old_keyboard_drive.cpp)
#rosbuild_link_boost(old_keyboard_drive signals)

# Keyboard drive
rosbuild_add_executable(keyboard_drive src/keyboard_drive.cpp)
rosbuild_link_boost(keyboard_drive signals)

# Sample acquisition node
rosbuild_add_executable(sample_acquisition src/sample_acquisition.cpp src/arm_drive.cpp src/stepper_helper.cpp)
rosbuild_link_boost(sample_acquisition signals)

#target_link_libraries(example ${PROJECT_NAME})
