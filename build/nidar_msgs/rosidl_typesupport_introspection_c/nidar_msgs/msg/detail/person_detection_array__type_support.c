// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from nidar_msgs:msg/PersonDetectionArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "nidar_msgs/msg/detail/person_detection_array__rosidl_typesupport_introspection_c.h"
#include "nidar_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "nidar_msgs/msg/detail/person_detection_array__functions.h"
#include "nidar_msgs/msg/detail/person_detection_array__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `detections`
#include "nidar_msgs/msg/person_detection.h"
// Member `detections`
#include "nidar_msgs/msg/detail/person_detection__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void nidar_msgs__msg__PersonDetectionArray__rosidl_typesupport_introspection_c__PersonDetectionArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  nidar_msgs__msg__PersonDetectionArray__init(message_memory);
}

void nidar_msgs__msg__PersonDetectionArray__rosidl_typesupport_introspection_c__PersonDetectionArray_fini_function(void * message_memory)
{
  nidar_msgs__msg__PersonDetectionArray__fini(message_memory);
}

size_t nidar_msgs__msg__PersonDetectionArray__rosidl_typesupport_introspection_c__size_function__PersonDetectionArray__detections(
  const void * untyped_member)
{
  const nidar_msgs__msg__PersonDetection__Sequence * member =
    (const nidar_msgs__msg__PersonDetection__Sequence *)(untyped_member);
  return member->size;
}

const void * nidar_msgs__msg__PersonDetectionArray__rosidl_typesupport_introspection_c__get_const_function__PersonDetectionArray__detections(
  const void * untyped_member, size_t index)
{
  const nidar_msgs__msg__PersonDetection__Sequence * member =
    (const nidar_msgs__msg__PersonDetection__Sequence *)(untyped_member);
  return &member->data[index];
}

void * nidar_msgs__msg__PersonDetectionArray__rosidl_typesupport_introspection_c__get_function__PersonDetectionArray__detections(
  void * untyped_member, size_t index)
{
  nidar_msgs__msg__PersonDetection__Sequence * member =
    (nidar_msgs__msg__PersonDetection__Sequence *)(untyped_member);
  return &member->data[index];
}

void nidar_msgs__msg__PersonDetectionArray__rosidl_typesupport_introspection_c__fetch_function__PersonDetectionArray__detections(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const nidar_msgs__msg__PersonDetection * item =
    ((const nidar_msgs__msg__PersonDetection *)
    nidar_msgs__msg__PersonDetectionArray__rosidl_typesupport_introspection_c__get_const_function__PersonDetectionArray__detections(untyped_member, index));
  nidar_msgs__msg__PersonDetection * value =
    (nidar_msgs__msg__PersonDetection *)(untyped_value);
  *value = *item;
}

void nidar_msgs__msg__PersonDetectionArray__rosidl_typesupport_introspection_c__assign_function__PersonDetectionArray__detections(
  void * untyped_member, size_t index, const void * untyped_value)
{
  nidar_msgs__msg__PersonDetection * item =
    ((nidar_msgs__msg__PersonDetection *)
    nidar_msgs__msg__PersonDetectionArray__rosidl_typesupport_introspection_c__get_function__PersonDetectionArray__detections(untyped_member, index));
  const nidar_msgs__msg__PersonDetection * value =
    (const nidar_msgs__msg__PersonDetection *)(untyped_value);
  *item = *value;
}

bool nidar_msgs__msg__PersonDetectionArray__rosidl_typesupport_introspection_c__resize_function__PersonDetectionArray__detections(
  void * untyped_member, size_t size)
{
  nidar_msgs__msg__PersonDetection__Sequence * member =
    (nidar_msgs__msg__PersonDetection__Sequence *)(untyped_member);
  nidar_msgs__msg__PersonDetection__Sequence__fini(member);
  return nidar_msgs__msg__PersonDetection__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember nidar_msgs__msg__PersonDetectionArray__rosidl_typesupport_introspection_c__PersonDetectionArray_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(nidar_msgs__msg__PersonDetectionArray, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "detections",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(nidar_msgs__msg__PersonDetectionArray, detections),  // bytes offset in struct
    NULL,  // default value
    nidar_msgs__msg__PersonDetectionArray__rosidl_typesupport_introspection_c__size_function__PersonDetectionArray__detections,  // size() function pointer
    nidar_msgs__msg__PersonDetectionArray__rosidl_typesupport_introspection_c__get_const_function__PersonDetectionArray__detections,  // get_const(index) function pointer
    nidar_msgs__msg__PersonDetectionArray__rosidl_typesupport_introspection_c__get_function__PersonDetectionArray__detections,  // get(index) function pointer
    nidar_msgs__msg__PersonDetectionArray__rosidl_typesupport_introspection_c__fetch_function__PersonDetectionArray__detections,  // fetch(index, &value) function pointer
    nidar_msgs__msg__PersonDetectionArray__rosidl_typesupport_introspection_c__assign_function__PersonDetectionArray__detections,  // assign(index, value) function pointer
    nidar_msgs__msg__PersonDetectionArray__rosidl_typesupport_introspection_c__resize_function__PersonDetectionArray__detections  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers nidar_msgs__msg__PersonDetectionArray__rosidl_typesupport_introspection_c__PersonDetectionArray_message_members = {
  "nidar_msgs__msg",  // message namespace
  "PersonDetectionArray",  // message name
  2,  // number of fields
  sizeof(nidar_msgs__msg__PersonDetectionArray),
  nidar_msgs__msg__PersonDetectionArray__rosidl_typesupport_introspection_c__PersonDetectionArray_message_member_array,  // message members
  nidar_msgs__msg__PersonDetectionArray__rosidl_typesupport_introspection_c__PersonDetectionArray_init_function,  // function to initialize message memory (memory has to be allocated)
  nidar_msgs__msg__PersonDetectionArray__rosidl_typesupport_introspection_c__PersonDetectionArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t nidar_msgs__msg__PersonDetectionArray__rosidl_typesupport_introspection_c__PersonDetectionArray_message_type_support_handle = {
  0,
  &nidar_msgs__msg__PersonDetectionArray__rosidl_typesupport_introspection_c__PersonDetectionArray_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_nidar_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, nidar_msgs, msg, PersonDetectionArray)() {
  nidar_msgs__msg__PersonDetectionArray__rosidl_typesupport_introspection_c__PersonDetectionArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  nidar_msgs__msg__PersonDetectionArray__rosidl_typesupport_introspection_c__PersonDetectionArray_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, nidar_msgs, msg, PersonDetection)();
  if (!nidar_msgs__msg__PersonDetectionArray__rosidl_typesupport_introspection_c__PersonDetectionArray_message_type_support_handle.typesupport_identifier) {
    nidar_msgs__msg__PersonDetectionArray__rosidl_typesupport_introspection_c__PersonDetectionArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &nidar_msgs__msg__PersonDetectionArray__rosidl_typesupport_introspection_c__PersonDetectionArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
