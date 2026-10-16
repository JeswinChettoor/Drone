// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from nidar_msgs:msg/PersonDetection.idl
// generated code does not contain a copyright notice

#ifndef NIDAR_MSGS__MSG__DETAIL__PERSON_DETECTION__STRUCT_H_
#define NIDAR_MSGS__MSG__DETAIL__PERSON_DETECTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/PersonDetection in the package nidar_msgs.
typedef struct nidar_msgs__msg__PersonDetection
{
  std_msgs__msg__Header header;
  float confidence;
  float x_min;
  float y_min;
  float x_max;
  float y_max;
} nidar_msgs__msg__PersonDetection;

// Struct for a sequence of nidar_msgs__msg__PersonDetection.
typedef struct nidar_msgs__msg__PersonDetection__Sequence
{
  nidar_msgs__msg__PersonDetection * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nidar_msgs__msg__PersonDetection__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // NIDAR_MSGS__MSG__DETAIL__PERSON_DETECTION__STRUCT_H_
