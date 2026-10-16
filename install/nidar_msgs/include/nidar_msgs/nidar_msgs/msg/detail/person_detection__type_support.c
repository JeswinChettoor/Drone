// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from nidar_msgs:msg/PersonDetection.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "nidar_msgs/msg/detail/person_detection__rosidl_typesupport_introspection_c.h"
#include "nidar_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "nidar_msgs/msg/detail/person_detection__functions.h"
#include "nidar_msgs/msg/detail/person_detection__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void nidar_msgs__msg__PersonDetection__rosidl_typesupport_introspection_c__PersonDetection_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  nidar_msgs__msg__PersonDetection__init(message_memory);
}

void nidar_msgs__msg__PersonDetection__rosidl_typesupport_introspection_c__PersonDetection_fini_function(void * message_memory)
{
  nidar_msgs__msg__PersonDetection__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember nidar_msgs__msg__PersonDetection__rosidl_typesupport_introspection_c__PersonDetection_message_member_array[6] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(nidar_msgs__msg__PersonDetection, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "confidence",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(nidar_msgs__msg__PersonDetection, confidence),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "x_min",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(nidar_msgs__msg__PersonDetection, x_min),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y_min",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(nidar_msgs__msg__PersonDetection, y_min),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "x_max",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(nidar_msgs__msg__PersonDetection, x_max),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y_max",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(nidar_msgs__msg__PersonDetection, y_max),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers nidar_msgs__msg__PersonDetection__rosidl_typesupport_introspection_c__PersonDetection_message_members = {
  "nidar_msgs__msg",  // message namespace
  "PersonDetection",  // message name
  6,  // number of fields
  sizeof(nidar_msgs__msg__PersonDetection),
  nidar_msgs__msg__PersonDetection__rosidl_typesupport_introspection_c__PersonDetection_message_member_array,  // message members
  nidar_msgs__msg__PersonDetection__rosidl_typesupport_introspection_c__PersonDetection_init_function,  // function to initialize message memory (memory has to be allocated)
  nidar_msgs__msg__PersonDetection__rosidl_typesupport_introspection_c__PersonDetection_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t nidar_msgs__msg__PersonDetection__rosidl_typesupport_introspection_c__PersonDetection_message_type_support_handle = {
  0,
  &nidar_msgs__msg__PersonDetection__rosidl_typesupport_introspection_c__PersonDetection_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_nidar_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, nidar_msgs, msg, PersonDetection)() {
  nidar_msgs__msg__PersonDetection__rosidl_typesupport_introspection_c__PersonDetection_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!nidar_msgs__msg__PersonDetection__rosidl_typesupport_introspection_c__PersonDetection_message_type_support_handle.typesupport_identifier) {
    nidar_msgs__msg__PersonDetection__rosidl_typesupport_introspection_c__PersonDetection_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &nidar_msgs__msg__PersonDetection__rosidl_typesupport_introspection_c__PersonDetection_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
