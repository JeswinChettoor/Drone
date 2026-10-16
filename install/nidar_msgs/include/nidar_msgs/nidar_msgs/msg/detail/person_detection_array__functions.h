// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from nidar_msgs:msg/PersonDetectionArray.idl
// generated code does not contain a copyright notice

#ifndef NIDAR_MSGS__MSG__DETAIL__PERSON_DETECTION_ARRAY__FUNCTIONS_H_
#define NIDAR_MSGS__MSG__DETAIL__PERSON_DETECTION_ARRAY__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "nidar_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "nidar_msgs/msg/detail/person_detection_array__struct.h"

/// Initialize msg/PersonDetectionArray message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * nidar_msgs__msg__PersonDetectionArray
 * )) before or use
 * nidar_msgs__msg__PersonDetectionArray__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_nidar_msgs
bool
nidar_msgs__msg__PersonDetectionArray__init(nidar_msgs__msg__PersonDetectionArray * msg);

/// Finalize msg/PersonDetectionArray message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_nidar_msgs
void
nidar_msgs__msg__PersonDetectionArray__fini(nidar_msgs__msg__PersonDetectionArray * msg);

/// Create msg/PersonDetectionArray message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * nidar_msgs__msg__PersonDetectionArray__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_nidar_msgs
nidar_msgs__msg__PersonDetectionArray *
nidar_msgs__msg__PersonDetectionArray__create();

/// Destroy msg/PersonDetectionArray message.
/**
 * It calls
 * nidar_msgs__msg__PersonDetectionArray__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_nidar_msgs
void
nidar_msgs__msg__PersonDetectionArray__destroy(nidar_msgs__msg__PersonDetectionArray * msg);

/// Check for msg/PersonDetectionArray message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_nidar_msgs
bool
nidar_msgs__msg__PersonDetectionArray__are_equal(const nidar_msgs__msg__PersonDetectionArray * lhs, const nidar_msgs__msg__PersonDetectionArray * rhs);

/// Copy a msg/PersonDetectionArray message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_nidar_msgs
bool
nidar_msgs__msg__PersonDetectionArray__copy(
  const nidar_msgs__msg__PersonDetectionArray * input,
  nidar_msgs__msg__PersonDetectionArray * output);

/// Initialize array of msg/PersonDetectionArray messages.
/**
 * It allocates the memory for the number of elements and calls
 * nidar_msgs__msg__PersonDetectionArray__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_nidar_msgs
bool
nidar_msgs__msg__PersonDetectionArray__Sequence__init(nidar_msgs__msg__PersonDetectionArray__Sequence * array, size_t size);

/// Finalize array of msg/PersonDetectionArray messages.
/**
 * It calls
 * nidar_msgs__msg__PersonDetectionArray__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_nidar_msgs
void
nidar_msgs__msg__PersonDetectionArray__Sequence__fini(nidar_msgs__msg__PersonDetectionArray__Sequence * array);

/// Create array of msg/PersonDetectionArray messages.
/**
 * It allocates the memory for the array and calls
 * nidar_msgs__msg__PersonDetectionArray__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_nidar_msgs
nidar_msgs__msg__PersonDetectionArray__Sequence *
nidar_msgs__msg__PersonDetectionArray__Sequence__create(size_t size);

/// Destroy array of msg/PersonDetectionArray messages.
/**
 * It calls
 * nidar_msgs__msg__PersonDetectionArray__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_nidar_msgs
void
nidar_msgs__msg__PersonDetectionArray__Sequence__destroy(nidar_msgs__msg__PersonDetectionArray__Sequence * array);

/// Check for msg/PersonDetectionArray message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_nidar_msgs
bool
nidar_msgs__msg__PersonDetectionArray__Sequence__are_equal(const nidar_msgs__msg__PersonDetectionArray__Sequence * lhs, const nidar_msgs__msg__PersonDetectionArray__Sequence * rhs);

/// Copy an array of msg/PersonDetectionArray messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_nidar_msgs
bool
nidar_msgs__msg__PersonDetectionArray__Sequence__copy(
  const nidar_msgs__msg__PersonDetectionArray__Sequence * input,
  nidar_msgs__msg__PersonDetectionArray__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // NIDAR_MSGS__MSG__DETAIL__PERSON_DETECTION_ARRAY__FUNCTIONS_H_
