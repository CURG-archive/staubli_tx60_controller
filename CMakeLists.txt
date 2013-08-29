cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

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
rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(soapC src/staubliTX60_api/soapC.cpp)
rosbuild_add_library(Server0 src/staubliTX60_api/soapCS8ServerV0Proxy.cpp)
rosbuild_add_library(Server1 src/staubliTX60_api/soapCS8ServerV1Proxy.cpp)
rosbuild_add_library(Server3 src/staubliTX60_api/soapCS8ServerV3Proxy.cpp)
rosbuild_add_library(stdsoap2 src/staubliTX60_api/stdsoap2.cpp)
rosbuild_add_library(tx60l src/staubliTX60_api/TX60L.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(staubliTX60_server src/staubliTX60_server.cpp)
rosbuild_add_executable(testing_client src/testing_client.cpp)
rosbuild_add_executable(wave src/wave.cpp)
target_link_libraries(staubliTX60_server tx60l)
target_link_libraries(staubliTX60_server soapC)
target_link_libraries(staubliTX60_server Server0)
target_link_libraries(staubliTX60_server Server1)
target_link_libraries(staubliTX60_server Server3)
target_link_libraries(staubliTX60_server stdsoap2)




