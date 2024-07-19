// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dhtt_msgs:msg/Resources.idl
// generated code does not contain a copyright notice
#include "dhtt_msgs/msg/detail/resources__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `resource_state`
#include "dhtt_msgs/msg/detail/resource__functions.h"

bool
dhtt_msgs__msg__Resources__init(dhtt_msgs__msg__Resources * msg)
{
  if (!msg) {
    return false;
  }
  // resource_state
  if (!dhtt_msgs__msg__Resource__Sequence__init(&msg->resource_state, 0)) {
    dhtt_msgs__msg__Resources__fini(msg);
    return false;
  }
  return true;
}

void
dhtt_msgs__msg__Resources__fini(dhtt_msgs__msg__Resources * msg)
{
  if (!msg) {
    return;
  }
  // resource_state
  dhtt_msgs__msg__Resource__Sequence__fini(&msg->resource_state);
}

bool
dhtt_msgs__msg__Resources__are_equal(const dhtt_msgs__msg__Resources * lhs, const dhtt_msgs__msg__Resources * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // resource_state
  if (!dhtt_msgs__msg__Resource__Sequence__are_equal(
      &(lhs->resource_state), &(rhs->resource_state)))
  {
    return false;
  }
  return true;
}

bool
dhtt_msgs__msg__Resources__copy(
  const dhtt_msgs__msg__Resources * input,
  dhtt_msgs__msg__Resources * output)
{
  if (!input || !output) {
    return false;
  }
  // resource_state
  if (!dhtt_msgs__msg__Resource__Sequence__copy(
      &(input->resource_state), &(output->resource_state)))
  {
    return false;
  }
  return true;
}

dhtt_msgs__msg__Resources *
dhtt_msgs__msg__Resources__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dhtt_msgs__msg__Resources * msg = (dhtt_msgs__msg__Resources *)allocator.allocate(sizeof(dhtt_msgs__msg__Resources), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dhtt_msgs__msg__Resources));
  bool success = dhtt_msgs__msg__Resources__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dhtt_msgs__msg__Resources__destroy(dhtt_msgs__msg__Resources * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dhtt_msgs__msg__Resources__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dhtt_msgs__msg__Resources__Sequence__init(dhtt_msgs__msg__Resources__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dhtt_msgs__msg__Resources * data = NULL;

  if (size) {
    data = (dhtt_msgs__msg__Resources *)allocator.zero_allocate(size, sizeof(dhtt_msgs__msg__Resources), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dhtt_msgs__msg__Resources__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dhtt_msgs__msg__Resources__fini(&data[i - 1]);
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
dhtt_msgs__msg__Resources__Sequence__fini(dhtt_msgs__msg__Resources__Sequence * array)
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
      dhtt_msgs__msg__Resources__fini(&array->data[i]);
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

dhtt_msgs__msg__Resources__Sequence *
dhtt_msgs__msg__Resources__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dhtt_msgs__msg__Resources__Sequence * array = (dhtt_msgs__msg__Resources__Sequence *)allocator.allocate(sizeof(dhtt_msgs__msg__Resources__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dhtt_msgs__msg__Resources__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dhtt_msgs__msg__Resources__Sequence__destroy(dhtt_msgs__msg__Resources__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dhtt_msgs__msg__Resources__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dhtt_msgs__msg__Resources__Sequence__are_equal(const dhtt_msgs__msg__Resources__Sequence * lhs, const dhtt_msgs__msg__Resources__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dhtt_msgs__msg__Resources__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dhtt_msgs__msg__Resources__Sequence__copy(
  const dhtt_msgs__msg__Resources__Sequence * input,
  dhtt_msgs__msg__Resources__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dhtt_msgs__msg__Resources);
    dhtt_msgs__msg__Resources * data =
      (dhtt_msgs__msg__Resources *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dhtt_msgs__msg__Resources__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          dhtt_msgs__msg__Resources__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dhtt_msgs__msg__Resources__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
