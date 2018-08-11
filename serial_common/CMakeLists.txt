cmake_minimum_required(VERSION 2.8.3)
project(serial_common)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   EnemyPos.msg
   Infantrymode.msg

   wifi_buff.msg
#   Message2.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here

generate_messages(
  DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES serial_common
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
)

add_executable(serial_common_node src/serial_common_node.cpp)
target_link_libraries(serial_common_node ${catkin_LIBRARIES})
