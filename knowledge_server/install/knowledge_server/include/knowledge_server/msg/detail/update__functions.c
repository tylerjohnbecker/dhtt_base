// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from knowledge_server:msg/Update.idl
// generated code does not contain a copyright notice
#include "knowledge_server/msg/detail/update__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `updated_pairs`
#include "knowledge_server/msg/detail/pair__functions.h"

bool
knowledge_server__msg__Update__init(knowledge_server__msg__Update * msg)
{
  if (!msg) {
    return false;
  }
  // updated_pairs
  if (!knowledge_server__msg__Pair__Sequence__init(&msg->updated_pairs, 0)) {
    knowledge_server__msg__Update__fini(msg);
    return false;
  }
  return true;
}

void
knowledge_server__msg__Update__fini(knowledge_server__msg__Update * msg)
{
  if (!msg) {
    return;
  }
  // updated_pairs
  knowledge_server__msg__Pair__Sequence__fini(&msg->updated_pairs);
}

bool
knowledge_server__msg__Update__are_equal(const knowledge_server__msg__Update * lhs, const knowledge_server__msg__Update * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // updated_pairs
  if (!knowledge_server__msg__Pair__Sequence__are_equal(
      &(lhs->updated_pairs), &(rhs->updated_pairs)))
  {
    return false;
  }
  return true;
}

bool
knowledge_server__msg__Update__copy(
  const knowledge_server__msg__Update * input,
  knowledge_server__msg__Update * output)
{
  if (!input || !output) {
    return false;
  }
  // updated_pairs
  if (!knowledge_server__msg__Pair__Sequence__copy(
      &(input->updated_pairs), &(output->updated_pairs)))
  {
    return false;
  }
  return true;
}

knowledge_server__msg__Update *
knowledge_server__msg__Update__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  knowledge_server__msg__Update * msg = (knowledge_server__msg__Update *)allocator.allocate(sizeof(knowledge_server__msg__Update), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(knowledge_server__msg__Update));
  bool success = knowledge_server__msg__Update__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
knowledge_server__msg__Update__destroy(knowledge_server__msg__Update * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    knowledge_server__msg__Update__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
knowledge_server__msg__Update__Sequence__init(knowledge_server__msg__Update__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  knowledge_server__msg__Update * data = NULL;

  if (size) {
    data = (knowledge_server__msg__Update *)allocator.zero_allocate(size, sizeof(knowledge_server__msg__Update), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = knowledge_server__msg__Update__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        knowledge_server__msg__Update__fini(&data[i - 1]);
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
knowledge_server__msg__Update__Sequence__fini(knowledge_server__msg__Update__Sequence * array)
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
      knowledge_server__msg__Update__fini(&array->data[i]);
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

knowledge_server__msg__Update__Sequence *
knowledge_server__msg__Update__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  knowledge_server__msg__Update__Sequence * array = (knowledge_server__msg__Update__Sequence *)allocator.allocate(sizeof(knowledge_server__msg__Update__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = knowledge_server__msg__Update__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
knowledge_server__msg__Update__Sequence__destroy(knowledge_server__msg__Update__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    knowledge_server__msg__Update__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
knowledge_server__msg__Update__Sequence__are_equal(const knowledge_server__msg__Update__Sequence * lhs, const knowledge_server__msg__Update__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!knowledge_server__msg__Update__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
knowledge_server__msg__Update__Sequence__copy(
  const knowledge_server__msg__Update__Sequence * input,
  knowledge_server__msg__Update__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(knowledge_server__msg__Update);
    knowledge_server__msg__Update * data =
      (knowledge_server__msg__Update *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!knowledge_server__msg__Update__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          knowledge_server__msg__Update__fini(&data[i]);
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
    if (!knowledge_server__msg__Update__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
