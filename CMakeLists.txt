cmake_minimum_required(VERSION 2.8.3)
project(ed_entity_server)

set(CMAKE_CXX_STANDARD 11)
find_package(catkin REQUIRED COMPONENTS
  ed
  geometry_msgs
  message_generation
  actionlib
)

add_action_files(
  FILES
    GetEntities.action
)

add_service_files(
  FILES
    SwitchPublisher.srv
)

generate_messages(
  DEPENDENCIES
    ed
    actionlib_msgs
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    ed
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(ed_entity_server_plugin
  src/entity_server_plugin.cpp
)
target_link_libraries(ed_entity_server_plugin
    ${catkin_LIBRARIES}
)
add_dependencies(ed_entity_server_plugin
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_generate_messages_cpp
)
