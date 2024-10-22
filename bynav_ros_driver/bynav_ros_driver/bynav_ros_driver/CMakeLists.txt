cmake_minimum_required(VERSION 3.0.2)
project(bynav_ros_driver)

add_compile_options(-std=c++11)

set(BUILD_DEPS
  nodelet
  roscpp
  std_msgs
  gps_common
  sensor_msgs
  nmea_msgs
  nav_msgs
  tf2_geometry_msgs
  message_generation
  novatel_oem7_msgs
)

find_package(catkin REQUIRED COMPONENTS ${BUILD_DEPS})
find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include
  src
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${novatel_oem7_msgs_INCLUDE_DIRS} 
	${PROJECT_SOURCE_DIR}/thirdparty/novatel_edie
	${PROJECT_SOURCE_DIR}/thirdparty/novatel_edie/src
	${PROJECT_SOURCE_DIR}/thirdparty/novatel_edie/usr/include/novatel/edie           
	${PROJECT_SOURCE_DIR}/thirdparty/novatel_edie/bin/Novatel/api/
	${PROJECT_SOURCE_DIR}/thirdparty/novatel_edie/bin/StreamInterface/api/
  ${PROJECT_SOURCE_DIR}/thirdparty/eigen
)

add_library(${PROJECT_NAME}
  src/oem7_log_nodelet.cpp
  src/oem7_message_nodelet.cpp
  src/oem7_config_nodelet.cpp
  src/oem7_receiver_net.cpp
  src/oem7_receiver_port.cpp
  src/oem7_message_decoder.cpp
  src/oem7_message_decoder_lib.cpp
  src/oem7_message_util.cpp
  src/oem7_ros_messages.cpp
  src/oem7_debug_file.cpp
  src/message_handler.cpp
  src/bestpos_handler.cpp
  src/ins_handler.cpp
  src/align_handler.cpp
  src/time_handler.cpp
  src/receiverstatus_handler.cpp
  src/nmea_handler.cpp
  src/oem7_imu.cpp
  src/tcp_client.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "armv7l")
  set(COMMON_LIB ${PROJECT_SOURCE_DIR}/thirdparty/novatel_edie/usr/lib/arm64/libCommon.a)
  set(STREAM_INTERFACE_LIB ${PROJECT_SOURCE_DIR}/thirdparty/novatel_edie/usr/lib/arm64/libStreamInterface.a)
  set(NOVATEL_LIB ${PROJECT_SOURCE_DIR}/thirdparty/novatel_edie/usr/lib/arm64/libNovatel.a)
  set(JSON_INTERFACE_LIB ${PROJECT_SOURCE_DIR}/thirdparty/novatel_edie/usr/lib/arm64/libJsonInterface.a)
else()
  set(COMMON_LIB ${PROJECT_SOURCE_DIR}/thirdparty/novatel_edie/usr/lib/x86/libCommon.a)
  set(STREAM_INTERFACE_LIB ${PROJECT_SOURCE_DIR}/thirdparty/novatel_edie/usr/lib/x86/libStreamInterface.a)
  set(NOVATEL_LIB ${PROJECT_SOURCE_DIR}/thirdparty/novatel_edie/usr/lib/x86/libNovatel.a)
  set(JSON_INTERFACE_LIB ${PROJECT_SOURCE_DIR}/thirdparty/novatel_edie/usr/lib/x86/libJsonInterface.a)
endif()

target_link_libraries(${PROJECT_NAME}
  Boost::boost
  ${STREAM_INTERFACE_LIB}
  ${NOVATEL_LIB}
  ${JSON_INTERFACE_LIB}
  ${COMMON_LIB}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}   
)

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Export plugin header files.
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
  FILES bynav_ros_driver_nodelets.xml bynav_ros_driver_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY 
  launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY 
  config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
