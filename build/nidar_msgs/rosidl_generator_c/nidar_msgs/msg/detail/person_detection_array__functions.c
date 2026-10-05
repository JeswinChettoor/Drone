// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from nidar_msgs:msg/PersonDetectionArray.idl
// generated code does not contain a copyright notice
#include "nidar_msgs/msg/detail/person_detection_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `detections`
#include "nidar_msgs/msg/detail/person_detection__functions.h"

bool
nidar_msgs__msg__PersonDetectionArray__init(nidar_msgs__msg__PersonDetectionArray * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    nidar_msgs__msg__PersonDetectionArray__fini(msg);
    return false;
  }
  // detections
  if (!nidar_msgs__msg__PersonDetection__Sequence__init(&msg->detections, 0)) {
    nidar_msgs__msg__PersonDetectionArray__fini(msg);
    return false;
  }
  return true;
}

void
nidar_msgs__msg__PersonDetectionArray__fini(nidar_msgs__msg__PersonDetectionArray * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // detections
  nidar_msgs__msg__PersonDetection__Sequence__fini(&msg->detections);
}

bool
nidar_msgs__msg__PersonDetectionArray__are_equal(const nidar_msgs__msg__PersonDetectionArray * lhs, const nidar_msgs__msg__PersonDetectionArray * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // detections
  if (!nidar_msgs__msg__PersonDetection__Sequence__are_equal(
      &(lhs->detections), &(rhs->detections)))
  {
    return false;
  }
  return true;
}

bool
nidar_msgs__msg__PersonDetectionArray__copy(
  const nidar_msgs__msg__PersonDetectionArray * input,
  nidar_msgs__msg__PersonDetectionArray * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // detections
  if (!nidar_msgs__msg__PersonDetection__Sequence__copy(
      &(input->detections), &(output->detections)))
  {
    return false;
  }
  return true;
}

nidar_msgs__msg__PersonDetectionArray *
nidar_msgs__msg__PersonDetectionArray__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nidar_msgs__msg__PersonDetectionArray * msg = (nidar_msgs__msg__PersonDetectionArray *)allocator.allocate(sizeof(nidar_msgs__msg__PersonDetectionArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(nidar_msgs__msg__PersonDetectionArray));
  bool success = nidar_msgs__msg__PersonDetectionArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
nidar_msgs__msg__PersonDetectionArray__destroy(nidar_msgs__msg__PersonDetectionArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    nidar_msgs__msg__PersonDetectionArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
nidar_msgs__msg__PersonDetectionArray__Sequence__init(nidar_msgs__msg__PersonDetectionArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nidar_msgs__msg__PersonDetectionArray * data = NULL;

  if (size) {
    data = (nidar_msgs__msg__PersonDetectionArray *)allocator.zero_allocate(size, sizeof(nidar_msgs__msg__PersonDetectionArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = nidar_msgs__msg__PersonDetectionArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        nidar_msgs__msg__PersonDetectionArray__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
nidar_msgs__msg__PersonDetectionArray__Sequence__fini(nidar_msgs__msg__PersonDetectionArray__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      nidar_msgs__msg__PersonDetectionArray__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

nidar_msgs__msg__PersonDetectionArray__Sequence *
nidar_msgs__msg__PersonDetectionArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nidar_msgs__msg__PersonDetectionArray__Sequence * array = (nidar_msgs__msg__PersonDetectionArray__Sequence *)allocator.allocate(sizeof(nidar_msgs__msg__PersonDetectionArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = nidar_msgs__msg__PersonDetectionArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
nidar_msgs__msg__PersonDetectionArray__Sequence__destroy(nidar_msgs__msg__PersonDetectionArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    nidar_msgs__msg__PersonDetectionArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
nidar_msgs__msg__PersonDetectionArray__Sequence__are_equal(const nidar_msgs__msg__PersonDetectionArray__Sequence * lhs, const nidar_msgs__msg__PersonDetectionArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!nidar_msgs__msg__PersonDetectionArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
nidar_msgs__msg__PersonDetectionArray__Sequence__copy(
  const nidar_msgs__msg__PersonDetectionArray__Sequence * input,
  nidar_msgs__msg__PersonDetectionArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(nidar_msgs__msg__PersonDetectionArray);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    nidar_msgs__msg__PersonDetectionArray * data =
      (nidar_msgs__msg__PersonDetectionArray *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!nidar_msgs__msg__PersonDetectionArray__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          nidar_msgs__msg__PersonDetectionArray__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!nidar_msgs__msg__PersonDetectionArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
