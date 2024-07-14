// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from knowledge_server:msg/Pair.idl
// generated code does not contain a copyright notice
#include "knowledge_server/msg/detail/pair__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `key`
// Member `value`
#include "rosidl_runtime_c/string_functions.h"

bool
knowledge_server__msg__Pair__init(knowledge_server__msg__Pair * msg)
{
  if (!msg) {
    return false;
  }
  // key
  if (!rosidl_runtime_c__String__init(&msg->key)) {
    knowledge_server__msg__Pair__fini(msg);
    return false;
  }
  // value
  if (!rosidl_runtime_c__String__init(&msg->value)) {
    knowledge_server__msg__Pair__fini(msg);
    return false;
  }
  return true;
}

void
knowledge_server__msg__Pair__fini(knowledge_server__msg__Pair * msg)
{
  if (!msg) {
    return;
  }
  // key
  rosidl_runtime_c__String__fini(&msg->key);
  // value
  rosidl_runtime_c__String__fini(&msg->value);
}

bool
knowledge_server__msg__Pair__are_equal(const knowledge_server__msg__Pair * lhs, const knowledge_server__msg__Pair * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // key
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->key), &(rhs->key)))
  {
    return false;
  }
  // value
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->value), &(rhs->value)))
  {
    return false;
  }
  return true;
}

bool
knowledge_server__msg__Pair__copy(
  const knowledge_server__msg__Pair * input,
  knowledge_server__msg__Pair * output)
{
  if (!input || !output) {
    return false;
  }
  // key
  if (!rosidl_runtime_c__String__copy(
      &(input->key), &(output->key)))
  {
    return false;
  }
  // value
  if (!rosidl_runtime_c__String__copy(
      &(input->value), &(output->value)))
  {
    return false;
  }
  return true;
}

knowledge_server__msg__Pair *
knowledge_server__msg__Pair__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  knowledge_server__msg__Pair * msg = (knowledge_server__msg__Pair *)allocator.allocate(sizeof(knowledge_server__msg__Pair), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(knowledge_server__msg__Pair));
  bool success = knowledge_server__msg__Pair__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
knowledge_server__msg__Pair__destroy(knowledge_server__msg__Pair * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    knowledge_server__msg__Pair__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
knowledge_server__msg__Pair__Sequence__init(knowledge_server__msg__Pair__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  knowledge_server__msg__Pair * data = NULL;

  if (size) {
    data = (knowledge_server__msg__Pair *)allocator.zero_allocate(size, sizeof(knowledge_server__msg__Pair), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = knowledge_server__msg__Pair__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        knowledge_server__msg__Pair__fini(&data[i - 1]);
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
knowledge_server__msg__Pair__Sequence__fini(knowledge_server__msg__Pair__Sequence * array)
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
      knowledge_server__msg__Pair__fini(&array->data[i]);
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

knowledge_server__msg__Pair__Sequence *
knowledge_server__msg__Pair__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  knowledge_server__msg__Pair__Sequence * array = (knowledge_server__msg__Pair__Sequence *)allocator.allocate(sizeof(knowledge_server__msg__Pair__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = knowledge_server__msg__Pair__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
knowledge_server__msg__Pair__Sequence__destroy(knowledge_server__msg__Pair__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    knowledge_server__msg__Pair__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
knowledge_server__msg__Pair__Sequence__are_equal(const knowledge_server__msg__Pair__Sequence * lhs, const knowledge_server__msg__Pair__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!knowledge_server__msg__Pair__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
knowledge_server__msg__Pair__Sequence__copy(
  const knowledge_server__msg__Pair__Sequence * input,
  knowledge_server__msg__Pair__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(knowledge_server__msg__Pair);
    knowledge_server__msg__Pair * data =
      (knowledge_server__msg__Pair *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!knowledge_server__msg__Pair__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          knowledge_server__msg__Pair__fini(&data[i]);
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
    if (!knowledge_server__msg__Pair__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
