cmake_minimum_required(VERSION 2.8.3)
project(hid)

find_package(catkin 
  REQUIRED COMPONENTS 
    std_msgs
    message_generation 
    roscpp 
    rospy 
)

find_package(Boost REQUIRED COMPONENTS system)

find_library(
  rnr_LIBRARIES2
  NAMES hid rnr
  PATHS /prj/lib /prj/lib/rnr /usr/local/lib /usr/local/lib/rnr
)

#catkin_python_setup()

add_message_files(
  DIRECTORY msg
  FILES
    Controller360State.msg
    LEDPattern.msg
    RumbleCmd.msg
)

add_service_files(
  DIRECTORY srv
  FILES
    Ping.srv
    SetLED.srv
    SetRumble.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

include_directories(include
  /prj/include
  /opt/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(xbox_360 src/xbox_360/xbox_360.cpp)

target_link_libraries( 
  xbox_360
  ${rnr_LIBRARIES2}
  ${catkin_LIBRARIES}
)

add_dependencies(xbox_360 hid_gencpp)

INSTALL(DIRECTORY www/img DESTINATION /opt/nginx/html/ros)
INSTALL(DIRECTORY www/js DESTINATION /opt/nginx/html/ros)
INSTALL(FILES www/html/ros/xbox360/index.html DESTINATION /opt/nginx/html/ros/xbox360)

