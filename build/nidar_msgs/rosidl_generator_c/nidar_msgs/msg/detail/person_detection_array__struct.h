// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from nidar_msgs:msg/PersonDetectionArray.idl
// generated code does not contain a copyright notice

#ifndef NIDAR_MSGS__MSG__DETAIL__PERSON_DETECTION_ARRAY__STRUCT_H_
#define NIDAR_MSGS__MSG__DETAIL__PERSON_DETECTION_ARRAY__STRUCT_H_

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
// Member 'detections'
#include "nidar_msgs/msg/detail/person_detection__struct.h"

/// Struct defined in msg/PersonDetectionArray in the package nidar_msgs.
typedef struct nidar_msgs__msg__PersonDetectionArray
{
  std_msgs__msg__Header header;
  nidar_msgs__msg__PersonDetection__Sequence detections;
} nidar_msgs__msg__PersonDetectionArray;

// Struct for a sequence of nidar_msgs__msg__PersonDetectionArray.
typedef struct nidar_msgs__msg__PersonDetectionArray__Sequence
{
  nidar_msgs__msg__PersonDetectionArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nidar_msgs__msg__PersonDetectionArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // NIDAR_MSGS__MSG__DETAIL__PERSON_DETECTION_ARRAY__STRUCT_H_
