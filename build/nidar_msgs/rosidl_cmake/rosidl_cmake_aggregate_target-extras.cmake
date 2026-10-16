# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target nidar_msgs::nidar_msgs
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${nidar_msgs_TARGETS}.
if(nidar_msgs_TARGETS AND NOT TARGET nidar_msgs::nidar_msgs)
  add_library(nidar_msgs::nidar_msgs INTERFACE IMPORTED)
  set_target_properties(nidar_msgs::nidar_msgs PROPERTIES
    INTERFACE_LINK_LIBRARIES "${nidar_msgs_TARGETS}")
endif()
