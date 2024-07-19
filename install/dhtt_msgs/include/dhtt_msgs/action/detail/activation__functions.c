// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dhtt_msgs:action/Activation.idl
// generated code does not contain a copyright notice
#include "dhtt_msgs/action/detail/activation__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `passed_resources`
// Member `granted_resources`
#include "dhtt_msgs/msg/detail/resource__functions.h"

bool
dhtt_msgs__action__Activation_Goal__init(dhtt_msgs__action__Activation_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // passed_resources
  if (!dhtt_msgs__msg__Resource__Sequence__init(&msg->passed_resources, 0)) {
    dhtt_msgs__action__Activation_Goal__fini(msg);
    return false;
  }
  // granted_resources
  if (!dhtt_msgs__msg__Resource__Sequence__init(&msg->granted_resources, 0)) {
    dhtt_msgs__action__Activation_Goal__fini(msg);
    return false;
  }
  // success
  return true;
}

void
dhtt_msgs__action__Activation_Goal__fini(dhtt_msgs__action__Activation_Goal * msg)
{
  if (!msg) {
    return;
  }
  // passed_resources
  dhtt_msgs__msg__Resource__Sequence__fini(&msg->passed_resources);
  // granted_resources
  dhtt_msgs__msg__Resource__Sequence__fini(&msg->granted_resources);
  // success
}

bool
dhtt_msgs__action__Activation_Goal__are_equal(const dhtt_msgs__action__Activation_Goal * lhs, const dhtt_msgs__action__Activation_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // passed_resources
  if (!dhtt_msgs__msg__Resource__Sequence__are_equal(
      &(lhs->passed_resources), &(rhs->passed_resources)))
  {
    return false;
  }
  // granted_resources
  if (!dhtt_msgs__msg__Resource__Sequence__are_equal(
      &(lhs->granted_resources), &(rhs->granted_resources)))
  {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
dhtt_msgs__action__Activation_Goal__copy(
  const dhtt_msgs__action__Activation_Goal * input,
  dhtt_msgs__action__Activation_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // passed_resources
  if (!dhtt_msgs__msg__Resource__Sequence__copy(
      &(input->passed_resources), &(output->passed_resources)))
  {
    return false;
  }
  // granted_resources
  if (!dhtt_msgs__msg__Resource__Sequence__copy(
      &(input->granted_resources), &(output->granted_resources)))
  {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

dhtt_msgs__action__Activation_Goal *
dhtt_msgs__action__Activation_Goal__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dhtt_msgs__action__Activation_Goal * msg = (dhtt_msgs__action__Activation_Goal *)allocator.allocate(sizeof(dhtt_msgs__action__Activation_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dhtt_msgs__action__Activation_Goal));
  bool success = dhtt_msgs__action__Activation_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dhtt_msgs__action__Activation_Goal__destroy(dhtt_msgs__action__Activation_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dhtt_msgs__action__Activation_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dhtt_msgs__action__Activation_Goal__Sequence__init(dhtt_msgs__action__Activation_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dhtt_msgs__action__Activation_Goal * data = NULL;

  if (size) {
    data = (dhtt_msgs__action__Activation_Goal *)allocator.zero_allocate(size, sizeof(dhtt_msgs__action__Activation_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dhtt_msgs__action__Activation_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dhtt_msgs__action__Activation_Goal__fini(&data[i - 1]);
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
dhtt_msgs__action__Activation_Goal__Sequence__fini(dhtt_msgs__action__Activation_Goal__Sequence * array)
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
      dhtt_msgs__action__Activation_Goal__fini(&array->data[i]);
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

dhtt_msgs__action__Activation_Goal__Sequence *
dhtt_msgs__action__Activation_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dhtt_msgs__action__Activation_Goal__Sequence * array = (dhtt_msgs__action__Activation_Goal__Sequence *)allocator.allocate(sizeof(dhtt_msgs__action__Activation_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dhtt_msgs__action__Activation_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dhtt_msgs__action__Activation_Goal__Sequence__destroy(dhtt_msgs__action__Activation_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dhtt_msgs__action__Activation_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dhtt_msgs__action__Activation_Goal__Sequence__are_equal(const dhtt_msgs__action__Activation_Goal__Sequence * lhs, const dhtt_msgs__action__Activation_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dhtt_msgs__action__Activation_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dhtt_msgs__action__Activation_Goal__Sequence__copy(
  const dhtt_msgs__action__Activation_Goal__Sequence * input,
  dhtt_msgs__action__Activation_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dhtt_msgs__action__Activation_Goal);
    dhtt_msgs__action__Activation_Goal * data =
      (dhtt_msgs__action__Activation_Goal *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dhtt_msgs__action__Activation_Goal__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          dhtt_msgs__action__Activation_Goal__fini(&data[i]);
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
    if (!dhtt_msgs__action__Activation_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `local_best_node`
#include "rosidl_runtime_c/string_functions.h"
// Member `requested_resources`
// Member `owned_resources`
// Member `released_resources`
// Member `passed_resources`
// already included above
// #include "dhtt_msgs/msg/detail/resource__functions.h"

bool
dhtt_msgs__action__Activation_Result__init(dhtt_msgs__action__Activation_Result * msg)
{
  if (!msg) {
    return false;
  }
  // local_best_node
  if (!rosidl_runtime_c__String__init(&msg->local_best_node)) {
    dhtt_msgs__action__Activation_Result__fini(msg);
    return false;
  }
  // requested_resources
  if (!dhtt_msgs__msg__Resource__Sequence__init(&msg->requested_resources, 0)) {
    dhtt_msgs__action__Activation_Result__fini(msg);
    return false;
  }
  // owned_resources
  if (!dhtt_msgs__msg__Resource__Sequence__init(&msg->owned_resources, 0)) {
    dhtt_msgs__action__Activation_Result__fini(msg);
    return false;
  }
  // done
  // possible
  // activation_potential
  // released_resources
  if (!dhtt_msgs__msg__Resource__Sequence__init(&msg->released_resources, 0)) {
    dhtt_msgs__action__Activation_Result__fini(msg);
    return false;
  }
  // passed_resources
  if (!dhtt_msgs__msg__Resource__Sequence__init(&msg->passed_resources, 0)) {
    dhtt_msgs__action__Activation_Result__fini(msg);
    return false;
  }
  return true;
}

void
dhtt_msgs__action__Activation_Result__fini(dhtt_msgs__action__Activation_Result * msg)
{
  if (!msg) {
    return;
  }
  // local_best_node
  rosidl_runtime_c__String__fini(&msg->local_best_node);
  // requested_resources
  dhtt_msgs__msg__Resource__Sequence__fini(&msg->requested_resources);
  // owned_resources
  dhtt_msgs__msg__Resource__Sequence__fini(&msg->owned_resources);
  // done
  // possible
  // activation_potential
  // released_resources
  dhtt_msgs__msg__Resource__Sequence__fini(&msg->released_resources);
  // passed_resources
  dhtt_msgs__msg__Resource__Sequence__fini(&msg->passed_resources);
}

bool
dhtt_msgs__action__Activation_Result__are_equal(const dhtt_msgs__action__Activation_Result * lhs, const dhtt_msgs__action__Activation_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // local_best_node
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->local_best_node), &(rhs->local_best_node)))
  {
    return false;
  }
  // requested_resources
  if (!dhtt_msgs__msg__Resource__Sequence__are_equal(
      &(lhs->requested_resources), &(rhs->requested_resources)))
  {
    return false;
  }
  // owned_resources
  if (!dhtt_msgs__msg__Resource__Sequence__are_equal(
      &(lhs->owned_resources), &(rhs->owned_resources)))
  {
    return false;
  }
  // done
  if (lhs->done != rhs->done) {
    return false;
  }
  // possible
  if (lhs->possible != rhs->possible) {
    return false;
  }
  // activation_potential
  if (lhs->activation_potential != rhs->activation_potential) {
    return false;
  }
  // released_resources
  if (!dhtt_msgs__msg__Resource__Sequence__are_equal(
      &(lhs->released_resources), &(rhs->released_resources)))
  {
    return false;
  }
  // passed_resources
  if (!dhtt_msgs__msg__Resource__Sequence__are_equal(
      &(lhs->passed_resources), &(rhs->passed_resources)))
  {
    return false;
  }
  return true;
}

bool
dhtt_msgs__action__Activation_Result__copy(
  const dhtt_msgs__action__Activation_Result * input,
  dhtt_msgs__action__Activation_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // local_best_node
  if (!rosidl_runtime_c__String__copy(
      &(input->local_best_node), &(output->local_best_node)))
  {
    return false;
  }
  // requested_resources
  if (!dhtt_msgs__msg__Resource__Sequence__copy(
      &(input->requested_resources), &(output->requested_resources)))
  {
    return false;
  }
  // owned_resources
  if (!dhtt_msgs__msg__Resource__Sequence__copy(
      &(input->owned_resources), &(output->owned_resources)))
  {
    return false;
  }
  // done
  output->done = input->done;
  // possible
  output->possible = input->possible;
  // activation_potential
  output->activation_potential = input->activation_potential;
  // released_resources
  if (!dhtt_msgs__msg__Resource__Sequence__copy(
      &(input->released_resources), &(output->released_resources)))
  {
    return false;
  }
  // passed_resources
  if (!dhtt_msgs__msg__Resource__Sequence__copy(
      &(input->passed_resources), &(output->passed_resources)))
  {
    return false;
  }
  return true;
}

dhtt_msgs__action__Activation_Result *
dhtt_msgs__action__Activation_Result__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dhtt_msgs__action__Activation_Result * msg = (dhtt_msgs__action__Activation_Result *)allocator.allocate(sizeof(dhtt_msgs__action__Activation_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dhtt_msgs__action__Activation_Result));
  bool success = dhtt_msgs__action__Activation_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dhtt_msgs__action__Activation_Result__destroy(dhtt_msgs__action__Activation_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dhtt_msgs__action__Activation_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dhtt_msgs__action__Activation_Result__Sequence__init(dhtt_msgs__action__Activation_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dhtt_msgs__action__Activation_Result * data = NULL;

  if (size) {
    data = (dhtt_msgs__action__Activation_Result *)allocator.zero_allocate(size, sizeof(dhtt_msgs__action__Activation_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dhtt_msgs__action__Activation_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dhtt_msgs__action__Activation_Result__fini(&data[i - 1]);
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
dhtt_msgs__action__Activation_Result__Sequence__fini(dhtt_msgs__action__Activation_Result__Sequence * array)
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
      dhtt_msgs__action__Activation_Result__fini(&array->data[i]);
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

dhtt_msgs__action__Activation_Result__Sequence *
dhtt_msgs__action__Activation_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dhtt_msgs__action__Activation_Result__Sequence * array = (dhtt_msgs__action__Activation_Result__Sequence *)allocator.allocate(sizeof(dhtt_msgs__action__Activation_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dhtt_msgs__action__Activation_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dhtt_msgs__action__Activation_Result__Sequence__destroy(dhtt_msgs__action__Activation_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dhtt_msgs__action__Activation_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dhtt_msgs__action__Activation_Result__Sequence__are_equal(const dhtt_msgs__action__Activation_Result__Sequence * lhs, const dhtt_msgs__action__Activation_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dhtt_msgs__action__Activation_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dhtt_msgs__action__Activation_Result__Sequence__copy(
  const dhtt_msgs__action__Activation_Result__Sequence * input,
  dhtt_msgs__action__Activation_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dhtt_msgs__action__Activation_Result);
    dhtt_msgs__action__Activation_Result * data =
      (dhtt_msgs__action__Activation_Result *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dhtt_msgs__action__Activation_Result__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          dhtt_msgs__action__Activation_Result__fini(&data[i]);
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
    if (!dhtt_msgs__action__Activation_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
dhtt_msgs__action__Activation_Feedback__init(dhtt_msgs__action__Activation_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
dhtt_msgs__action__Activation_Feedback__fini(dhtt_msgs__action__Activation_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
dhtt_msgs__action__Activation_Feedback__are_equal(const dhtt_msgs__action__Activation_Feedback * lhs, const dhtt_msgs__action__Activation_Feedback * rhs)
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
dhtt_msgs__action__Activation_Feedback__copy(
  const dhtt_msgs__action__Activation_Feedback * input,
  dhtt_msgs__action__Activation_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

dhtt_msgs__action__Activation_Feedback *
dhtt_msgs__action__Activation_Feedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dhtt_msgs__action__Activation_Feedback * msg = (dhtt_msgs__action__Activation_Feedback *)allocator.allocate(sizeof(dhtt_msgs__action__Activation_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dhtt_msgs__action__Activation_Feedback));
  bool success = dhtt_msgs__action__Activation_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dhtt_msgs__action__Activation_Feedback__destroy(dhtt_msgs__action__Activation_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dhtt_msgs__action__Activation_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dhtt_msgs__action__Activation_Feedback__Sequence__init(dhtt_msgs__action__Activation_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dhtt_msgs__action__Activation_Feedback * data = NULL;

  if (size) {
    data = (dhtt_msgs__action__Activation_Feedback *)allocator.zero_allocate(size, sizeof(dhtt_msgs__action__Activation_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dhtt_msgs__action__Activation_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dhtt_msgs__action__Activation_Feedback__fini(&data[i - 1]);
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
dhtt_msgs__action__Activation_Feedback__Sequence__fini(dhtt_msgs__action__Activation_Feedback__Sequence * array)
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
      dhtt_msgs__action__Activation_Feedback__fini(&array->data[i]);
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

dhtt_msgs__action__Activation_Feedback__Sequence *
dhtt_msgs__action__Activation_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dhtt_msgs__action__Activation_Feedback__Sequence * array = (dhtt_msgs__action__Activation_Feedback__Sequence *)allocator.allocate(sizeof(dhtt_msgs__action__Activation_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dhtt_msgs__action__Activation_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dhtt_msgs__action__Activation_Feedback__Sequence__destroy(dhtt_msgs__action__Activation_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dhtt_msgs__action__Activation_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dhtt_msgs__action__Activation_Feedback__Sequence__are_equal(const dhtt_msgs__action__Activation_Feedback__Sequence * lhs, const dhtt_msgs__action__Activation_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dhtt_msgs__action__Activation_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dhtt_msgs__action__Activation_Feedback__Sequence__copy(
  const dhtt_msgs__action__Activation_Feedback__Sequence * input,
  dhtt_msgs__action__Activation_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dhtt_msgs__action__Activation_Feedback);
    dhtt_msgs__action__Activation_Feedback * data =
      (dhtt_msgs__action__Activation_Feedback *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dhtt_msgs__action__Activation_Feedback__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          dhtt_msgs__action__Activation_Feedback__fini(&data[i]);
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
    if (!dhtt_msgs__action__Activation_Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "dhtt_msgs/action/detail/activation__functions.h"

bool
dhtt_msgs__action__Activation_SendGoal_Request__init(dhtt_msgs__action__Activation_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    dhtt_msgs__action__Activation_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!dhtt_msgs__action__Activation_Goal__init(&msg->goal)) {
    dhtt_msgs__action__Activation_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
dhtt_msgs__action__Activation_SendGoal_Request__fini(dhtt_msgs__action__Activation_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  dhtt_msgs__action__Activation_Goal__fini(&msg->goal);
}

bool
dhtt_msgs__action__Activation_SendGoal_Request__are_equal(const dhtt_msgs__action__Activation_SendGoal_Request * lhs, const dhtt_msgs__action__Activation_SendGoal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // goal
  if (!dhtt_msgs__action__Activation_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
dhtt_msgs__action__Activation_SendGoal_Request__copy(
  const dhtt_msgs__action__Activation_SendGoal_Request * input,
  dhtt_msgs__action__Activation_SendGoal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // goal
  if (!dhtt_msgs__action__Activation_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

dhtt_msgs__action__Activation_SendGoal_Request *
dhtt_msgs__action__Activation_SendGoal_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dhtt_msgs__action__Activation_SendGoal_Request * msg = (dhtt_msgs__action__Activation_SendGoal_Request *)allocator.allocate(sizeof(dhtt_msgs__action__Activation_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dhtt_msgs__action__Activation_SendGoal_Request));
  bool success = dhtt_msgs__action__Activation_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dhtt_msgs__action__Activation_SendGoal_Request__destroy(dhtt_msgs__action__Activation_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dhtt_msgs__action__Activation_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dhtt_msgs__action__Activation_SendGoal_Request__Sequence__init(dhtt_msgs__action__Activation_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dhtt_msgs__action__Activation_SendGoal_Request * data = NULL;

  if (size) {
    data = (dhtt_msgs__action__Activation_SendGoal_Request *)allocator.zero_allocate(size, sizeof(dhtt_msgs__action__Activation_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dhtt_msgs__action__Activation_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dhtt_msgs__action__Activation_SendGoal_Request__fini(&data[i - 1]);
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
dhtt_msgs__action__Activation_SendGoal_Request__Sequence__fini(dhtt_msgs__action__Activation_SendGoal_Request__Sequence * array)
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
      dhtt_msgs__action__Activation_SendGoal_Request__fini(&array->data[i]);
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

dhtt_msgs__action__Activation_SendGoal_Request__Sequence *
dhtt_msgs__action__Activation_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dhtt_msgs__action__Activation_SendGoal_Request__Sequence * array = (dhtt_msgs__action__Activation_SendGoal_Request__Sequence *)allocator.allocate(sizeof(dhtt_msgs__action__Activation_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dhtt_msgs__action__Activation_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dhtt_msgs__action__Activation_SendGoal_Request__Sequence__destroy(dhtt_msgs__action__Activation_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dhtt_msgs__action__Activation_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dhtt_msgs__action__Activation_SendGoal_Request__Sequence__are_equal(const dhtt_msgs__action__Activation_SendGoal_Request__Sequence * lhs, const dhtt_msgs__action__Activation_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dhtt_msgs__action__Activation_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dhtt_msgs__action__Activation_SendGoal_Request__Sequence__copy(
  const dhtt_msgs__action__Activation_SendGoal_Request__Sequence * input,
  dhtt_msgs__action__Activation_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dhtt_msgs__action__Activation_SendGoal_Request);
    dhtt_msgs__action__Activation_SendGoal_Request * data =
      (dhtt_msgs__action__Activation_SendGoal_Request *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dhtt_msgs__action__Activation_SendGoal_Request__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          dhtt_msgs__action__Activation_SendGoal_Request__fini(&data[i]);
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
    if (!dhtt_msgs__action__Activation_SendGoal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
dhtt_msgs__action__Activation_SendGoal_Response__init(dhtt_msgs__action__Activation_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    dhtt_msgs__action__Activation_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
dhtt_msgs__action__Activation_SendGoal_Response__fini(dhtt_msgs__action__Activation_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
dhtt_msgs__action__Activation_SendGoal_Response__are_equal(const dhtt_msgs__action__Activation_SendGoal_Response * lhs, const dhtt_msgs__action__Activation_SendGoal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
dhtt_msgs__action__Activation_SendGoal_Response__copy(
  const dhtt_msgs__action__Activation_SendGoal_Response * input,
  dhtt_msgs__action__Activation_SendGoal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

dhtt_msgs__action__Activation_SendGoal_Response *
dhtt_msgs__action__Activation_SendGoal_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dhtt_msgs__action__Activation_SendGoal_Response * msg = (dhtt_msgs__action__Activation_SendGoal_Response *)allocator.allocate(sizeof(dhtt_msgs__action__Activation_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dhtt_msgs__action__Activation_SendGoal_Response));
  bool success = dhtt_msgs__action__Activation_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dhtt_msgs__action__Activation_SendGoal_Response__destroy(dhtt_msgs__action__Activation_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dhtt_msgs__action__Activation_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dhtt_msgs__action__Activation_SendGoal_Response__Sequence__init(dhtt_msgs__action__Activation_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dhtt_msgs__action__Activation_SendGoal_Response * data = NULL;

  if (size) {
    data = (dhtt_msgs__action__Activation_SendGoal_Response *)allocator.zero_allocate(size, sizeof(dhtt_msgs__action__Activation_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dhtt_msgs__action__Activation_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dhtt_msgs__action__Activation_SendGoal_Response__fini(&data[i - 1]);
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
dhtt_msgs__action__Activation_SendGoal_Response__Sequence__fini(dhtt_msgs__action__Activation_SendGoal_Response__Sequence * array)
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
      dhtt_msgs__action__Activation_SendGoal_Response__fini(&array->data[i]);
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

dhtt_msgs__action__Activation_SendGoal_Response__Sequence *
dhtt_msgs__action__Activation_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dhtt_msgs__action__Activation_SendGoal_Response__Sequence * array = (dhtt_msgs__action__Activation_SendGoal_Response__Sequence *)allocator.allocate(sizeof(dhtt_msgs__action__Activation_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dhtt_msgs__action__Activation_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dhtt_msgs__action__Activation_SendGoal_Response__Sequence__destroy(dhtt_msgs__action__Activation_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dhtt_msgs__action__Activation_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dhtt_msgs__action__Activation_SendGoal_Response__Sequence__are_equal(const dhtt_msgs__action__Activation_SendGoal_Response__Sequence * lhs, const dhtt_msgs__action__Activation_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dhtt_msgs__action__Activation_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dhtt_msgs__action__Activation_SendGoal_Response__Sequence__copy(
  const dhtt_msgs__action__Activation_SendGoal_Response__Sequence * input,
  dhtt_msgs__action__Activation_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dhtt_msgs__action__Activation_SendGoal_Response);
    dhtt_msgs__action__Activation_SendGoal_Response * data =
      (dhtt_msgs__action__Activation_SendGoal_Response *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dhtt_msgs__action__Activation_SendGoal_Response__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          dhtt_msgs__action__Activation_SendGoal_Response__fini(&data[i]);
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
    if (!dhtt_msgs__action__Activation_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
dhtt_msgs__action__Activation_GetResult_Request__init(dhtt_msgs__action__Activation_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    dhtt_msgs__action__Activation_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
dhtt_msgs__action__Activation_GetResult_Request__fini(dhtt_msgs__action__Activation_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
dhtt_msgs__action__Activation_GetResult_Request__are_equal(const dhtt_msgs__action__Activation_GetResult_Request * lhs, const dhtt_msgs__action__Activation_GetResult_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  return true;
}

bool
dhtt_msgs__action__Activation_GetResult_Request__copy(
  const dhtt_msgs__action__Activation_GetResult_Request * input,
  dhtt_msgs__action__Activation_GetResult_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  return true;
}

dhtt_msgs__action__Activation_GetResult_Request *
dhtt_msgs__action__Activation_GetResult_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dhtt_msgs__action__Activation_GetResult_Request * msg = (dhtt_msgs__action__Activation_GetResult_Request *)allocator.allocate(sizeof(dhtt_msgs__action__Activation_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dhtt_msgs__action__Activation_GetResult_Request));
  bool success = dhtt_msgs__action__Activation_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dhtt_msgs__action__Activation_GetResult_Request__destroy(dhtt_msgs__action__Activation_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dhtt_msgs__action__Activation_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dhtt_msgs__action__Activation_GetResult_Request__Sequence__init(dhtt_msgs__action__Activation_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dhtt_msgs__action__Activation_GetResult_Request * data = NULL;

  if (size) {
    data = (dhtt_msgs__action__Activation_GetResult_Request *)allocator.zero_allocate(size, sizeof(dhtt_msgs__action__Activation_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dhtt_msgs__action__Activation_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dhtt_msgs__action__Activation_GetResult_Request__fini(&data[i - 1]);
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
dhtt_msgs__action__Activation_GetResult_Request__Sequence__fini(dhtt_msgs__action__Activation_GetResult_Request__Sequence * array)
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
      dhtt_msgs__action__Activation_GetResult_Request__fini(&array->data[i]);
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

dhtt_msgs__action__Activation_GetResult_Request__Sequence *
dhtt_msgs__action__Activation_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dhtt_msgs__action__Activation_GetResult_Request__Sequence * array = (dhtt_msgs__action__Activation_GetResult_Request__Sequence *)allocator.allocate(sizeof(dhtt_msgs__action__Activation_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dhtt_msgs__action__Activation_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dhtt_msgs__action__Activation_GetResult_Request__Sequence__destroy(dhtt_msgs__action__Activation_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dhtt_msgs__action__Activation_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dhtt_msgs__action__Activation_GetResult_Request__Sequence__are_equal(const dhtt_msgs__action__Activation_GetResult_Request__Sequence * lhs, const dhtt_msgs__action__Activation_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dhtt_msgs__action__Activation_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dhtt_msgs__action__Activation_GetResult_Request__Sequence__copy(
  const dhtt_msgs__action__Activation_GetResult_Request__Sequence * input,
  dhtt_msgs__action__Activation_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dhtt_msgs__action__Activation_GetResult_Request);
    dhtt_msgs__action__Activation_GetResult_Request * data =
      (dhtt_msgs__action__Activation_GetResult_Request *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dhtt_msgs__action__Activation_GetResult_Request__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          dhtt_msgs__action__Activation_GetResult_Request__fini(&data[i]);
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
    if (!dhtt_msgs__action__Activation_GetResult_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
// already included above
// #include "dhtt_msgs/action/detail/activation__functions.h"

bool
dhtt_msgs__action__Activation_GetResult_Response__init(dhtt_msgs__action__Activation_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!dhtt_msgs__action__Activation_Result__init(&msg->result)) {
    dhtt_msgs__action__Activation_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
dhtt_msgs__action__Activation_GetResult_Response__fini(dhtt_msgs__action__Activation_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  dhtt_msgs__action__Activation_Result__fini(&msg->result);
}

bool
dhtt_msgs__action__Activation_GetResult_Response__are_equal(const dhtt_msgs__action__Activation_GetResult_Response * lhs, const dhtt_msgs__action__Activation_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!dhtt_msgs__action__Activation_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
dhtt_msgs__action__Activation_GetResult_Response__copy(
  const dhtt_msgs__action__Activation_GetResult_Response * input,
  dhtt_msgs__action__Activation_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!dhtt_msgs__action__Activation_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

dhtt_msgs__action__Activation_GetResult_Response *
dhtt_msgs__action__Activation_GetResult_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dhtt_msgs__action__Activation_GetResult_Response * msg = (dhtt_msgs__action__Activation_GetResult_Response *)allocator.allocate(sizeof(dhtt_msgs__action__Activation_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dhtt_msgs__action__Activation_GetResult_Response));
  bool success = dhtt_msgs__action__Activation_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dhtt_msgs__action__Activation_GetResult_Response__destroy(dhtt_msgs__action__Activation_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dhtt_msgs__action__Activation_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dhtt_msgs__action__Activation_GetResult_Response__Sequence__init(dhtt_msgs__action__Activation_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dhtt_msgs__action__Activation_GetResult_Response * data = NULL;

  if (size) {
    data = (dhtt_msgs__action__Activation_GetResult_Response *)allocator.zero_allocate(size, sizeof(dhtt_msgs__action__Activation_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dhtt_msgs__action__Activation_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dhtt_msgs__action__Activation_GetResult_Response__fini(&data[i - 1]);
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
dhtt_msgs__action__Activation_GetResult_Response__Sequence__fini(dhtt_msgs__action__Activation_GetResult_Response__Sequence * array)
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
      dhtt_msgs__action__Activation_GetResult_Response__fini(&array->data[i]);
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

dhtt_msgs__action__Activation_GetResult_Response__Sequence *
dhtt_msgs__action__Activation_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dhtt_msgs__action__Activation_GetResult_Response__Sequence * array = (dhtt_msgs__action__Activation_GetResult_Response__Sequence *)allocator.allocate(sizeof(dhtt_msgs__action__Activation_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dhtt_msgs__action__Activation_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dhtt_msgs__action__Activation_GetResult_Response__Sequence__destroy(dhtt_msgs__action__Activation_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dhtt_msgs__action__Activation_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dhtt_msgs__action__Activation_GetResult_Response__Sequence__are_equal(const dhtt_msgs__action__Activation_GetResult_Response__Sequence * lhs, const dhtt_msgs__action__Activation_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dhtt_msgs__action__Activation_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dhtt_msgs__action__Activation_GetResult_Response__Sequence__copy(
  const dhtt_msgs__action__Activation_GetResult_Response__Sequence * input,
  dhtt_msgs__action__Activation_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dhtt_msgs__action__Activation_GetResult_Response);
    dhtt_msgs__action__Activation_GetResult_Response * data =
      (dhtt_msgs__action__Activation_GetResult_Response *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dhtt_msgs__action__Activation_GetResult_Response__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          dhtt_msgs__action__Activation_GetResult_Response__fini(&data[i]);
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
    if (!dhtt_msgs__action__Activation_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "dhtt_msgs/action/detail/activation__functions.h"

bool
dhtt_msgs__action__Activation_FeedbackMessage__init(dhtt_msgs__action__Activation_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    dhtt_msgs__action__Activation_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!dhtt_msgs__action__Activation_Feedback__init(&msg->feedback)) {
    dhtt_msgs__action__Activation_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
dhtt_msgs__action__Activation_FeedbackMessage__fini(dhtt_msgs__action__Activation_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  dhtt_msgs__action__Activation_Feedback__fini(&msg->feedback);
}

bool
dhtt_msgs__action__Activation_FeedbackMessage__are_equal(const dhtt_msgs__action__Activation_FeedbackMessage * lhs, const dhtt_msgs__action__Activation_FeedbackMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // feedback
  if (!dhtt_msgs__action__Activation_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
dhtt_msgs__action__Activation_FeedbackMessage__copy(
  const dhtt_msgs__action__Activation_FeedbackMessage * input,
  dhtt_msgs__action__Activation_FeedbackMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // feedback
  if (!dhtt_msgs__action__Activation_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

dhtt_msgs__action__Activation_FeedbackMessage *
dhtt_msgs__action__Activation_FeedbackMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dhtt_msgs__action__Activation_FeedbackMessage * msg = (dhtt_msgs__action__Activation_FeedbackMessage *)allocator.allocate(sizeof(dhtt_msgs__action__Activation_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dhtt_msgs__action__Activation_FeedbackMessage));
  bool success = dhtt_msgs__action__Activation_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dhtt_msgs__action__Activation_FeedbackMessage__destroy(dhtt_msgs__action__Activation_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dhtt_msgs__action__Activation_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dhtt_msgs__action__Activation_FeedbackMessage__Sequence__init(dhtt_msgs__action__Activation_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dhtt_msgs__action__Activation_FeedbackMessage * data = NULL;

  if (size) {
    data = (dhtt_msgs__action__Activation_FeedbackMessage *)allocator.zero_allocate(size, sizeof(dhtt_msgs__action__Activation_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dhtt_msgs__action__Activation_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dhtt_msgs__action__Activation_FeedbackMessage__fini(&data[i - 1]);
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
dhtt_msgs__action__Activation_FeedbackMessage__Sequence__fini(dhtt_msgs__action__Activation_FeedbackMessage__Sequence * array)
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
      dhtt_msgs__action__Activation_FeedbackMessage__fini(&array->data[i]);
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

dhtt_msgs__action__Activation_FeedbackMessage__Sequence *
dhtt_msgs__action__Activation_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dhtt_msgs__action__Activation_FeedbackMessage__Sequence * array = (dhtt_msgs__action__Activation_FeedbackMessage__Sequence *)allocator.allocate(sizeof(dhtt_msgs__action__Activation_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dhtt_msgs__action__Activation_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dhtt_msgs__action__Activation_FeedbackMessage__Sequence__destroy(dhtt_msgs__action__Activation_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dhtt_msgs__action__Activation_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dhtt_msgs__action__Activation_FeedbackMessage__Sequence__are_equal(const dhtt_msgs__action__Activation_FeedbackMessage__Sequence * lhs, const dhtt_msgs__action__Activation_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dhtt_msgs__action__Activation_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dhtt_msgs__action__Activation_FeedbackMessage__Sequence__copy(
  const dhtt_msgs__action__Activation_FeedbackMessage__Sequence * input,
  dhtt_msgs__action__Activation_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dhtt_msgs__action__Activation_FeedbackMessage);
    dhtt_msgs__action__Activation_FeedbackMessage * data =
      (dhtt_msgs__action__Activation_FeedbackMessage *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dhtt_msgs__action__Activation_FeedbackMessage__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          dhtt_msgs__action__Activation_FeedbackMessage__fini(&data[i]);
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
    if (!dhtt_msgs__action__Activation_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
