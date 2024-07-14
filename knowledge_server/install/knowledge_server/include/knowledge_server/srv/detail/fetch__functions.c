// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from knowledge_server:srv/Fetch.idl
// generated code does not contain a copyright notice
#include "knowledge_server/srv/detail/fetch__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `keys`
#include "rosidl_runtime_c/string_functions.h"

bool
knowledge_server__srv__Fetch_Request__init(knowledge_server__srv__Fetch_Request * msg)
{
  if (!msg) {
    return false;
  }
  // keys
  if (!rosidl_runtime_c__String__Sequence__init(&msg->keys, 0)) {
    knowledge_server__srv__Fetch_Request__fini(msg);
    return false;
  }
  return true;
}

void
knowledge_server__srv__Fetch_Request__fini(knowledge_server__srv__Fetch_Request * msg)
{
  if (!msg) {
    return;
  }
  // keys
  rosidl_runtime_c__String__Sequence__fini(&msg->keys);
}

bool
knowledge_server__srv__Fetch_Request__are_equal(const knowledge_server__srv__Fetch_Request * lhs, const knowledge_server__srv__Fetch_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // keys
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->keys), &(rhs->keys)))
  {
    return false;
  }
  return true;
}

bool
knowledge_server__srv__Fetch_Request__copy(
  const knowledge_server__srv__Fetch_Request * input,
  knowledge_server__srv__Fetch_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // keys
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->keys), &(output->keys)))
  {
    return false;
  }
  return true;
}

knowledge_server__srv__Fetch_Request *
knowledge_server__srv__Fetch_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  knowledge_server__srv__Fetch_Request * msg = (knowledge_server__srv__Fetch_Request *)allocator.allocate(sizeof(knowledge_server__srv__Fetch_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(knowledge_server__srv__Fetch_Request));
  bool success = knowledge_server__srv__Fetch_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
knowledge_server__srv__Fetch_Request__destroy(knowledge_server__srv__Fetch_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    knowledge_server__srv__Fetch_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
knowledge_server__srv__Fetch_Request__Sequence__init(knowledge_server__srv__Fetch_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  knowledge_server__srv__Fetch_Request * data = NULL;

  if (size) {
    data = (knowledge_server__srv__Fetch_Request *)allocator.zero_allocate(size, sizeof(knowledge_server__srv__Fetch_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = knowledge_server__srv__Fetch_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        knowledge_server__srv__Fetch_Request__fini(&data[i - 1]);
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
knowledge_server__srv__Fetch_Request__Sequence__fini(knowledge_server__srv__Fetch_Request__Sequence * array)
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
      knowledge_server__srv__Fetch_Request__fini(&array->data[i]);
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

knowledge_server__srv__Fetch_Request__Sequence *
knowledge_server__srv__Fetch_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  knowledge_server__srv__Fetch_Request__Sequence * array = (knowledge_server__srv__Fetch_Request__Sequence *)allocator.allocate(sizeof(knowledge_server__srv__Fetch_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = knowledge_server__srv__Fetch_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
knowledge_server__srv__Fetch_Request__Sequence__destroy(knowledge_server__srv__Fetch_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    knowledge_server__srv__Fetch_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
knowledge_server__srv__Fetch_Request__Sequence__are_equal(const knowledge_server__srv__Fetch_Request__Sequence * lhs, const knowledge_server__srv__Fetch_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!knowledge_server__srv__Fetch_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
knowledge_server__srv__Fetch_Request__Sequence__copy(
  const knowledge_server__srv__Fetch_Request__Sequence * input,
  knowledge_server__srv__Fetch_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(knowledge_server__srv__Fetch_Request);
    knowledge_server__srv__Fetch_Request * data =
      (knowledge_server__srv__Fetch_Request *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!knowledge_server__srv__Fetch_Request__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          knowledge_server__srv__Fetch_Request__fini(&data[i]);
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
    if (!knowledge_server__srv__Fetch_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `pairs`
#include "knowledge_server/msg/detail/pair__functions.h"

bool
knowledge_server__srv__Fetch_Response__init(knowledge_server__srv__Fetch_Response * msg)
{
  if (!msg) {
    return false;
  }
  // pairs
  if (!knowledge_server__msg__Pair__Sequence__init(&msg->pairs, 0)) {
    knowledge_server__srv__Fetch_Response__fini(msg);
    return false;
  }
  return true;
}

void
knowledge_server__srv__Fetch_Response__fini(knowledge_server__srv__Fetch_Response * msg)
{
  if (!msg) {
    return;
  }
  // pairs
  knowledge_server__msg__Pair__Sequence__fini(&msg->pairs);
}

bool
knowledge_server__srv__Fetch_Response__are_equal(const knowledge_server__srv__Fetch_Response * lhs, const knowledge_server__srv__Fetch_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // pairs
  if (!knowledge_server__msg__Pair__Sequence__are_equal(
      &(lhs->pairs), &(rhs->pairs)))
  {
    return false;
  }
  return true;
}

bool
knowledge_server__srv__Fetch_Response__copy(
  const knowledge_server__srv__Fetch_Response * input,
  knowledge_server__srv__Fetch_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // pairs
  if (!knowledge_server__msg__Pair__Sequence__copy(
      &(input->pairs), &(output->pairs)))
  {
    return false;
  }
  return true;
}

knowledge_server__srv__Fetch_Response *
knowledge_server__srv__Fetch_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  knowledge_server__srv__Fetch_Response * msg = (knowledge_server__srv__Fetch_Response *)allocator.allocate(sizeof(knowledge_server__srv__Fetch_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(knowledge_server__srv__Fetch_Response));
  bool success = knowledge_server__srv__Fetch_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
knowledge_server__srv__Fetch_Response__destroy(knowledge_server__srv__Fetch_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    knowledge_server__srv__Fetch_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
knowledge_server__srv__Fetch_Response__Sequence__init(knowledge_server__srv__Fetch_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  knowledge_server__srv__Fetch_Response * data = NULL;

  if (size) {
    data = (knowledge_server__srv__Fetch_Response *)allocator.zero_allocate(size, sizeof(knowledge_server__srv__Fetch_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = knowledge_server__srv__Fetch_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        knowledge_server__srv__Fetch_Response__fini(&data[i - 1]);
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
knowledge_server__srv__Fetch_Response__Sequence__fini(knowledge_server__srv__Fetch_Response__Sequence * array)
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
      knowledge_server__srv__Fetch_Response__fini(&array->data[i]);
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

knowledge_server__srv__Fetch_Response__Sequence *
knowledge_server__srv__Fetch_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  knowledge_server__srv__Fetch_Response__Sequence * array = (knowledge_server__srv__Fetch_Response__Sequence *)allocator.allocate(sizeof(knowledge_server__srv__Fetch_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = knowledge_server__srv__Fetch_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
knowledge_server__srv__Fetch_Response__Sequence__destroy(knowledge_server__srv__Fetch_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    knowledge_server__srv__Fetch_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
knowledge_server__srv__Fetch_Response__Sequence__are_equal(const knowledge_server__srv__Fetch_Response__Sequence * lhs, const knowledge_server__srv__Fetch_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!knowledge_server__srv__Fetch_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
knowledge_server__srv__Fetch_Response__Sequence__copy(
  const knowledge_server__srv__Fetch_Response__Sequence * input,
  knowledge_server__srv__Fetch_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(knowledge_server__srv__Fetch_Response);
    knowledge_server__srv__Fetch_Response * data =
      (knowledge_server__srv__Fetch_Response *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!knowledge_server__srv__Fetch_Response__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          knowledge_server__srv__Fetch_Response__fini(&data[i]);
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
    if (!knowledge_server__srv__Fetch_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
