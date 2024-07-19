// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dhtt_msgs:msg/Subtree.idl
// generated code does not contain a copyright notice
#include "dhtt_msgs/msg/detail/subtree__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `tree_nodes`
#include "dhtt_msgs/msg/detail/node__functions.h"

bool
dhtt_msgs__msg__Subtree__init(dhtt_msgs__msg__Subtree * msg)
{
  if (!msg) {
    return false;
  }
  // tree_nodes
  if (!dhtt_msgs__msg__Node__Sequence__init(&msg->tree_nodes, 0)) {
    dhtt_msgs__msg__Subtree__fini(msg);
    return false;
  }
  // tree_status
  // max_tree_depth
  // max_tree_width
  // task_completion_percent
  return true;
}

void
dhtt_msgs__msg__Subtree__fini(dhtt_msgs__msg__Subtree * msg)
{
  if (!msg) {
    return;
  }
  // tree_nodes
  dhtt_msgs__msg__Node__Sequence__fini(&msg->tree_nodes);
  // tree_status
  // max_tree_depth
  // max_tree_width
  // task_completion_percent
}

bool
dhtt_msgs__msg__Subtree__are_equal(const dhtt_msgs__msg__Subtree * lhs, const dhtt_msgs__msg__Subtree * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // tree_nodes
  if (!dhtt_msgs__msg__Node__Sequence__are_equal(
      &(lhs->tree_nodes), &(rhs->tree_nodes)))
  {
    return false;
  }
  // tree_status
  if (lhs->tree_status != rhs->tree_status) {
    return false;
  }
  // max_tree_depth
  if (lhs->max_tree_depth != rhs->max_tree_depth) {
    return false;
  }
  // max_tree_width
  if (lhs->max_tree_width != rhs->max_tree_width) {
    return false;
  }
  // task_completion_percent
  if (lhs->task_completion_percent != rhs->task_completion_percent) {
    return false;
  }
  return true;
}

bool
dhtt_msgs__msg__Subtree__copy(
  const dhtt_msgs__msg__Subtree * input,
  dhtt_msgs__msg__Subtree * output)
{
  if (!input || !output) {
    return false;
  }
  // tree_nodes
  if (!dhtt_msgs__msg__Node__Sequence__copy(
      &(input->tree_nodes), &(output->tree_nodes)))
  {
    return false;
  }
  // tree_status
  output->tree_status = input->tree_status;
  // max_tree_depth
  output->max_tree_depth = input->max_tree_depth;
  // max_tree_width
  output->max_tree_width = input->max_tree_width;
  // task_completion_percent
  output->task_completion_percent = input->task_completion_percent;
  return true;
}

dhtt_msgs__msg__Subtree *
dhtt_msgs__msg__Subtree__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dhtt_msgs__msg__Subtree * msg = (dhtt_msgs__msg__Subtree *)allocator.allocate(sizeof(dhtt_msgs__msg__Subtree), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dhtt_msgs__msg__Subtree));
  bool success = dhtt_msgs__msg__Subtree__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dhtt_msgs__msg__Subtree__destroy(dhtt_msgs__msg__Subtree * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dhtt_msgs__msg__Subtree__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dhtt_msgs__msg__Subtree__Sequence__init(dhtt_msgs__msg__Subtree__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dhtt_msgs__msg__Subtree * data = NULL;

  if (size) {
    data = (dhtt_msgs__msg__Subtree *)allocator.zero_allocate(size, sizeof(dhtt_msgs__msg__Subtree), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dhtt_msgs__msg__Subtree__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dhtt_msgs__msg__Subtree__fini(&data[i - 1]);
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
dhtt_msgs__msg__Subtree__Sequence__fini(dhtt_msgs__msg__Subtree__Sequence * array)
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
      dhtt_msgs__msg__Subtree__fini(&array->data[i]);
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

dhtt_msgs__msg__Subtree__Sequence *
dhtt_msgs__msg__Subtree__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dhtt_msgs__msg__Subtree__Sequence * array = (dhtt_msgs__msg__Subtree__Sequence *)allocator.allocate(sizeof(dhtt_msgs__msg__Subtree__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dhtt_msgs__msg__Subtree__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dhtt_msgs__msg__Subtree__Sequence__destroy(dhtt_msgs__msg__Subtree__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dhtt_msgs__msg__Subtree__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dhtt_msgs__msg__Subtree__Sequence__are_equal(const dhtt_msgs__msg__Subtree__Sequence * lhs, const dhtt_msgs__msg__Subtree__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dhtt_msgs__msg__Subtree__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dhtt_msgs__msg__Subtree__Sequence__copy(
  const dhtt_msgs__msg__Subtree__Sequence * input,
  dhtt_msgs__msg__Subtree__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dhtt_msgs__msg__Subtree);
    dhtt_msgs__msg__Subtree * data =
      (dhtt_msgs__msg__Subtree *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dhtt_msgs__msg__Subtree__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          dhtt_msgs__msg__Subtree__fini(&data[i]);
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
    if (!dhtt_msgs__msg__Subtree__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
