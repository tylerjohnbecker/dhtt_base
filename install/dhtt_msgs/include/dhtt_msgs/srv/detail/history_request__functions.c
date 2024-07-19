// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dhtt_msgs:srv/HistoryRequest.idl
// generated code does not contain a copyright notice
#include "dhtt_msgs/srv/detail/history_request__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
dhtt_msgs__srv__HistoryRequest_Request__init(dhtt_msgs__srv__HistoryRequest_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
dhtt_msgs__srv__HistoryRequest_Request__fini(dhtt_msgs__srv__HistoryRequest_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
dhtt_msgs__srv__HistoryRequest_Request__are_equal(const dhtt_msgs__srv__HistoryRequest_Request * lhs, const dhtt_msgs__srv__HistoryRequest_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
dhtt_msgs__srv__HistoryRequest_Request__copy(
  const dhtt_msgs__srv__HistoryRequest_Request * input,
  dhtt_msgs__srv__HistoryRequest_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

dhtt_msgs__srv__HistoryRequest_Request *
dhtt_msgs__srv__HistoryRequest_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dhtt_msgs__srv__HistoryRequest_Request * msg = (dhtt_msgs__srv__HistoryRequest_Request *)allocator.allocate(sizeof(dhtt_msgs__srv__HistoryRequest_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dhtt_msgs__srv__HistoryRequest_Request));
  bool success = dhtt_msgs__srv__HistoryRequest_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dhtt_msgs__srv__HistoryRequest_Request__destroy(dhtt_msgs__srv__HistoryRequest_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dhtt_msgs__srv__HistoryRequest_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dhtt_msgs__srv__HistoryRequest_Request__Sequence__init(dhtt_msgs__srv__HistoryRequest_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dhtt_msgs__srv__HistoryRequest_Request * data = NULL;

  if (size) {
    data = (dhtt_msgs__srv__HistoryRequest_Request *)allocator.zero_allocate(size, sizeof(dhtt_msgs__srv__HistoryRequest_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dhtt_msgs__srv__HistoryRequest_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dhtt_msgs__srv__HistoryRequest_Request__fini(&data[i - 1]);
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
dhtt_msgs__srv__HistoryRequest_Request__Sequence__fini(dhtt_msgs__srv__HistoryRequest_Request__Sequence * array)
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
      dhtt_msgs__srv__HistoryRequest_Request__fini(&array->data[i]);
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

dhtt_msgs__srv__HistoryRequest_Request__Sequence *
dhtt_msgs__srv__HistoryRequest_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dhtt_msgs__srv__HistoryRequest_Request__Sequence * array = (dhtt_msgs__srv__HistoryRequest_Request__Sequence *)allocator.allocate(sizeof(dhtt_msgs__srv__HistoryRequest_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dhtt_msgs__srv__HistoryRequest_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dhtt_msgs__srv__HistoryRequest_Request__Sequence__destroy(dhtt_msgs__srv__HistoryRequest_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dhtt_msgs__srv__HistoryRequest_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dhtt_msgs__srv__HistoryRequest_Request__Sequence__are_equal(const dhtt_msgs__srv__HistoryRequest_Request__Sequence * lhs, const dhtt_msgs__srv__HistoryRequest_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dhtt_msgs__srv__HistoryRequest_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dhtt_msgs__srv__HistoryRequest_Request__Sequence__copy(
  const dhtt_msgs__srv__HistoryRequest_Request__Sequence * input,
  dhtt_msgs__srv__HistoryRequest_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dhtt_msgs__srv__HistoryRequest_Request);
    dhtt_msgs__srv__HistoryRequest_Request * data =
      (dhtt_msgs__srv__HistoryRequest_Request *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dhtt_msgs__srv__HistoryRequest_Request__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          dhtt_msgs__srv__HistoryRequest_Request__fini(&data[i]);
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
    if (!dhtt_msgs__srv__HistoryRequest_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `node_history`
#include "rosidl_runtime_c/string_functions.h"

bool
dhtt_msgs__srv__HistoryRequest_Response__init(dhtt_msgs__srv__HistoryRequest_Response * msg)
{
  if (!msg) {
    return false;
  }
  // node_history
  if (!rosidl_runtime_c__String__Sequence__init(&msg->node_history, 0)) {
    dhtt_msgs__srv__HistoryRequest_Response__fini(msg);
    return false;
  }
  return true;
}

void
dhtt_msgs__srv__HistoryRequest_Response__fini(dhtt_msgs__srv__HistoryRequest_Response * msg)
{
  if (!msg) {
    return;
  }
  // node_history
  rosidl_runtime_c__String__Sequence__fini(&msg->node_history);
}

bool
dhtt_msgs__srv__HistoryRequest_Response__are_equal(const dhtt_msgs__srv__HistoryRequest_Response * lhs, const dhtt_msgs__srv__HistoryRequest_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // node_history
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->node_history), &(rhs->node_history)))
  {
    return false;
  }
  return true;
}

bool
dhtt_msgs__srv__HistoryRequest_Response__copy(
  const dhtt_msgs__srv__HistoryRequest_Response * input,
  dhtt_msgs__srv__HistoryRequest_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // node_history
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->node_history), &(output->node_history)))
  {
    return false;
  }
  return true;
}

dhtt_msgs__srv__HistoryRequest_Response *
dhtt_msgs__srv__HistoryRequest_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dhtt_msgs__srv__HistoryRequest_Response * msg = (dhtt_msgs__srv__HistoryRequest_Response *)allocator.allocate(sizeof(dhtt_msgs__srv__HistoryRequest_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dhtt_msgs__srv__HistoryRequest_Response));
  bool success = dhtt_msgs__srv__HistoryRequest_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dhtt_msgs__srv__HistoryRequest_Response__destroy(dhtt_msgs__srv__HistoryRequest_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dhtt_msgs__srv__HistoryRequest_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dhtt_msgs__srv__HistoryRequest_Response__Sequence__init(dhtt_msgs__srv__HistoryRequest_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dhtt_msgs__srv__HistoryRequest_Response * data = NULL;

  if (size) {
    data = (dhtt_msgs__srv__HistoryRequest_Response *)allocator.zero_allocate(size, sizeof(dhtt_msgs__srv__HistoryRequest_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dhtt_msgs__srv__HistoryRequest_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dhtt_msgs__srv__HistoryRequest_Response__fini(&data[i - 1]);
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
dhtt_msgs__srv__HistoryRequest_Response__Sequence__fini(dhtt_msgs__srv__HistoryRequest_Response__Sequence * array)
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
      dhtt_msgs__srv__HistoryRequest_Response__fini(&array->data[i]);
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

dhtt_msgs__srv__HistoryRequest_Response__Sequence *
dhtt_msgs__srv__HistoryRequest_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dhtt_msgs__srv__HistoryRequest_Response__Sequence * array = (dhtt_msgs__srv__HistoryRequest_Response__Sequence *)allocator.allocate(sizeof(dhtt_msgs__srv__HistoryRequest_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dhtt_msgs__srv__HistoryRequest_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dhtt_msgs__srv__HistoryRequest_Response__Sequence__destroy(dhtt_msgs__srv__HistoryRequest_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dhtt_msgs__srv__HistoryRequest_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dhtt_msgs__srv__HistoryRequest_Response__Sequence__are_equal(const dhtt_msgs__srv__HistoryRequest_Response__Sequence * lhs, const dhtt_msgs__srv__HistoryRequest_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dhtt_msgs__srv__HistoryRequest_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dhtt_msgs__srv__HistoryRequest_Response__Sequence__copy(
  const dhtt_msgs__srv__HistoryRequest_Response__Sequence * input,
  dhtt_msgs__srv__HistoryRequest_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dhtt_msgs__srv__HistoryRequest_Response);
    dhtt_msgs__srv__HistoryRequest_Response * data =
      (dhtt_msgs__srv__HistoryRequest_Response *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dhtt_msgs__srv__HistoryRequest_Response__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          dhtt_msgs__srv__HistoryRequest_Response__fini(&data[i]);
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
    if (!dhtt_msgs__srv__HistoryRequest_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
