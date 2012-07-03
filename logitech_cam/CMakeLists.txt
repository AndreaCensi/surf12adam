cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

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
# The add executable line has to come before the link libraries
rosbuild_add_executable(cam_actuator_node src/cam_actuator_node.cc src/CreeperCam.cc src/controls.cc)
target_link_libraries(cam_actuator_node /usr/lib/libwebcam.so)
target_link_libraries(cam_actuator_node /usr/lib/libwebcam.so.0)
target_link_libraries(cam_actuator_node /usr/lib/libwebcam.so.0.2.1)

rosbuild_add_executable(ctrl_prompt_node src/ctrl_prompt_node.cc src/CreeperCam.cc src/controls.cc)
#rosbuild_add_boost_directories()
target_link_libraries(ctrl_prompt_node /usr/lib/libwebcam.so)
target_link_libraries(ctrl_prompt_node /usr/lib/libwebcam.so.0)
target_link_libraries(ctrl_prompt_node /usr/lib/libwebcam.so.0.2.1)