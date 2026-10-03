# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target ardupilot_msgs::ardupilot_msgs
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${ardupilot_msgs_TARGETS}.
if(ardupilot_msgs_TARGETS AND NOT TARGET ardupilot_msgs::ardupilot_msgs)
  add_library(ardupilot_msgs::ardupilot_msgs INTERFACE IMPORTED)
  set_target_properties(ardupilot_msgs::ardupilot_msgs PROPERTIES
    INTERFACE_LINK_LIBRARIES "${ardupilot_msgs_TARGETS}")
endif()
