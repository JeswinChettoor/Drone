// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from nidar_msgs:msg/PersonDetection.idl
// generated code does not contain a copyright notice
#include "nidar_msgs/msg/detail/person_detection__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
nidar_msgs__msg__PersonDetection__init(nidar_msgs__msg__PersonDetection * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    nidar_msgs__msg__PersonDetection__fini(msg);
    return false;
  }
  // confidence
  // x_min
  // y_min
  // x_max
  // y_max
  return true;
}

void
nidar_msgs__msg__PersonDetection__fini(nidar_msgs__msg__PersonDetection * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // confidence
  // x_min
  // y_min
  // x_max
  // y_max
}

bool
nidar_msgs__msg__PersonDetection__are_equal(const nidar_msgs__msg__PersonDetection * lhs, const nidar_msgs__msg__PersonDetection * rhs)
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
  // confidence
  if (lhs->confidence != rhs->confidence) {
    return false;
  }
  // x_min
  if (lhs->x_min != rhs->x_min) {
    return false;
  }
  // y_min
  if (lhs->y_min != rhs->y_min) {
    return false;
  }
  // x_max
  if (lhs->x_max != rhs->x_max) {
    return false;
  }
  // y_max
  if (lhs->y_max != rhs->y_max) {
    return false;
  }
  return true;
}

bool
nidar_msgs__msg__PersonDetection__copy(
  const nidar_msgs__msg__PersonDetection * input,
  nidar_msgs__msg__PersonDetection * output)
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
  // confidence
  output->confidence = input->confidence;
  // x_min
  output->x_min = input->x_min;
  // y_min
  output->y_min = input->y_min;
  // x_max
  output->x_max = input->x_max;
  // y_max
  output->y_max = input->y_max;
  return true;
}

nidar_msgs__msg__PersonDetection *
nidar_msgs__msg__PersonDetection__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nidar_msgs__msg__PersonDetection * msg = (nidar_msgs__msg__PersonDetection *)allocator.allocate(sizeof(nidar_msgs__msg__PersonDetection), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(nidar_msgs__msg__PersonDetection));
  bool success = nidar_msgs__msg__PersonDetection__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
nidar_msgs__msg__PersonDetection__destroy(nidar_msgs__msg__PersonDetection * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    nidar_msgs__msg__PersonDetection__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
nidar_msgs__msg__PersonDetection__Sequence__init(nidar_msgs__msg__PersonDetection__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nidar_msgs__msg__PersonDetection * data = NULL;

  if (size) {
    data = (nidar_msgs__msg__PersonDetection *)allocator.zero_allocate(size, sizeof(nidar_msgs__msg__PersonDetection), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = nidar_msgs__msg__PersonDetection__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        nidar_msgs__msg__PersonDetection__fini(&data[i - 1]);
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
nidar_msgs__msg__PersonDetection__Sequence__fini(nidar_msgs__msg__PersonDetection__Sequence * array)
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
      nidar_msgs__msg__PersonDetection__fini(&array->data[i]);
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

nidar_msgs__msg__PersonDetection__Sequence *
nidar_msgs__msg__PersonDetection__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nidar_msgs__msg__PersonDetection__Sequence * array = (nidar_msgs__msg__PersonDetection__Sequence *)allocator.allocate(sizeof(nidar_msgs__msg__PersonDetection__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = nidar_msgs__msg__PersonDetection__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
nidar_msgs__msg__PersonDetection__Sequence__destroy(nidar_msgs__msg__PersonDetection__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    nidar_msgs__msg__PersonDetection__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
nidar_msgs__msg__PersonDetection__Sequence__are_equal(const nidar_msgs__msg__PersonDetection__Sequence * lhs, const nidar_msgs__msg__PersonDetection__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!nidar_msgs__msg__PersonDetection__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
nidar_msgs__msg__PersonDetection__Sequence__copy(
  const nidar_msgs__msg__PersonDetection__Sequence * input,
  nidar_msgs__msg__PersonDetection__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(nidar_msgs__msg__PersonDetection);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    nidar_msgs__msg__PersonDetection * data =
      (nidar_msgs__msg__PersonDetection *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!nidar_msgs__msg__PersonDetection__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          nidar_msgs__msg__PersonDetection__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!nidar_msgs__msg__PersonDetection__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
