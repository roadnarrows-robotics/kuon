cmake_minimum_required(VERSION 2.8.3)
project(kuon_control)

find_package(catkin 
  REQUIRED COMPONENTS 
  std_msgs 
  genmsg 
  message_generation 
  roscpp 
  rospy 
)

find_package(Boost REQUIRED COMPONENTS system)

# RDK, DHP TODO Create rnrpkgConfig.cmake
# find_package(rnr
#   REQUIRED COMPONENTS hekateros Dynamixel botsense netmsgs rnr_appkit rnr
# WARNING:  The rnr libraries must of an unique name or other packages will
#           clobber.
# )
find_library(
  kuon_ctl_LIBRARIES
  NAMES RS160D serial rnr 
  PATHS /prj/lib /prj/lib/rnr
)

message("hey")
message(${kuon_ctl_LIBRARIES})
message("oooooooooooooooooooooo")

#catkin_python_setup()

add_message_files(
  DIRECTORY msg
  FILES 
    KuonState.msg
    KuonStatus.msg
    MotorCommand.msg
    Version.msg
)

add_service_files(
  DIRECTORY srv
  FILES
    EStop.srv
    QueryVersion.srv
    ResetEStop.srv
)

## Generate added messages/services/actions with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS)

add_executable(kuon_control 
                 src/kuon_control_main.cpp 
                 src/kuon_control.cpp 
                 src/KuonRobot.cpp)

include_directories(
  include 
  /prj/include 
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

target_link_libraries(kuon_control 
  ${kuon_ctl_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(kuon_control kuon_control_gencpp)

#install(
#PROGRAMS 
#scripts/joint_trajectory_demo 
#DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

