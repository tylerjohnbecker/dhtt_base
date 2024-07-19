// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dhtt_msgs:msg/Node.idl
// generated code does not contain a copyright notice
#include "dhtt_msgs/msg/detail/node__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `head`
#include "std_msgs/msg/detail/header__functions.h"
// Member `node_name`
// Member `parent_name`
// Member `child_name`
// Member `params`
// Member `plugin_name`
#include "rosidl_runtime_c/string_functions.h"
// Member `children`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `owned_resources`
#include "dhtt_msgs/msg/detail/resource__functions.h"
// Member `node_status`
#include "dhtt_msgs/msg/detail/node_status__functions.h"

bool
dhtt_msgs__msg__Node__init(dhtt_msgs__msg__Node * msg)
{
  if (!msg) {
    return false;
  }
  // head
  if (!std_msgs__msg__Header__init(&msg->head)) {
    dhtt_msgs__msg__Node__fini(msg);
    return false;
  }
  // node_name
  if (!rosidl_runtime_c__String__init(&msg->node_name)) {
    dhtt_msgs__msg__Node__fini(msg);
    return false;
  }
  // parent
  // parent_name
  if (!rosidl_runtime_c__String__init(&msg->parent_name)) {
    dhtt_msgs__msg__Node__fini(msg);
    return false;
  }
  // children
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->children, 0)) {
    dhtt_msgs__msg__Node__fini(msg);
    return false;
  }
  // child_name
  if (!rosidl_runtime_c__String__Sequence__init(&msg->child_name, 0)) {
    dhtt_msgs__msg__Node__fini(msg);
    return false;
  }
  // params
  if (!rosidl_runtime_c__String__Sequence__init(&msg->params, 0)) {
    dhtt_msgs__msg__Node__fini(msg);
    return false;
  }
  // type
  // plugin_name
  if (!rosidl_runtime_c__String__init(&msg->plugin_name)) {
    dhtt_msgs__msg__Node__fini(msg);
    return false;
  }
  // owned_resources
  if (!dhtt_msgs__msg__Resource__Sequence__init(&msg->owned_resources, 0)) {
    dhtt_msgs__msg__Node__fini(msg);
    return false;
  }
  // node_status
  if (!dhtt_msgs__msg__NodeStatus__init(&msg->node_status)) {
    dhtt_msgs__msg__Node__fini(msg);
    return false;
  }
  return true;
}

void
dhtt_msgs__msg__Node__fini(dhtt_msgs__msg__Node * msg)
{
  if (!msg) {
    return;
  }
  // head
  std_msgs__msg__Header__fini(&msg->head);
  // node_name
  rosidl_runtime_c__String__fini(&msg->node_name);
  // parent
  // parent_name
  rosidl_runtime_c__String__fini(&msg->parent_name);
  // children
  rosidl_runtime_c__int32__Sequence__fini(&msg->children);
  // child_name
  rosidl_runtime_c__String__Sequence__fini(&msg->child_name);
  // params
  rosidl_runtime_c__String__Sequence__fini(&msg->params);
  // type
  // plugin_name
  rosidl_runtime_c__String__fini(&msg->plugin_name);
  // owned_resources
  dhtt_msgs__msg__Resource__Sequence__fini(&msg->owned_resources);
  // node_status
  dhtt_msgs__msg__NodeStatus__fini(&msg->node_status);
}

bool
dhtt_msgs__msg__Node__are_equal(const dhtt_msgs__msg__Node * lhs, const dhtt_msgs__msg__Node * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // head
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->head), &(rhs->head)))
  {
    return false;
  }
  // node_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->node_name), &(rhs->node_name)))
  {
    return false;
  }
  // parent
  if (lhs->parent != rhs->parent) {
    return false;
  }
  // parent_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->parent_name), &(rhs->parent_name)))
  {
    return false;
  }
  // children
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->children), &(rhs->children)))
  {
    return false;
  }
  // child_name
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->child_name), &(rhs->child_name)))
  {
    return false;
  }
  // params
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->params), &(rhs->params)))
  {
    return false;
  }
  // type
  if (lhs->type != rhs->type) {
    return false;
  }
  // plugin_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->plugin_name), &(rhs->plugin_name)))
  {
    return false;
  }
  // owned_resources
  if (!dhtt_msgs__msg__Resource__Sequence__are_equal(
      &(lhs->owned_resources), &(rhs->owned_resources)))
  {
    return false;
  }
  // node_status
  if (!dhtt_msgs__msg__NodeStatus__are_equal(
      &(lhs->node_status), &(rhs->node_status)))
  {
    return false;
  }
  return true;
}

bool
dhtt_msgs__msg__Node__copy(
  const dhtt_msgs__msg__Node * input,
  dhtt_msgs__msg__Node * output)
{
  if (!input || !output) {
    return false;
  }
  // head
  if (!std_msgs__msg__Header__copy(
      &(input->head), &(output->head)))
  {
    return false;
  }
  // node_name
  if (!rosidl_runtime_c__String__copy(
      &(input->node_name), &(output->node_name)))
  {
    return false;
  }
  // parent
  output->parent = input->parent;
  // parent_name
  if (!rosidl_runtime_c__String__copy(
      &(input->parent_name), &(output->parent_name)))
  {
    return false;
  }
  // children
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->children), &(output->children)))
  {
    return false;
  }
  // child_name
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->child_name), &(output->child_name)))
  {
    return false;
  }
  // params
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->params), &(output->params)))
  {
    return false;
  }
  // type
  output->type = input->type;
  // plugin_name
  if (!rosidl_runtime_c__String__copy(
      &(input->plugin_name), &(output->plugin_name)))
  {
    return false;
  }
  // owned_resources
  if (!dhtt_msgs__msg__Resource__Sequence__copy(
      &(input->owned_resources), &(output->owned_resources)))
  {
    return false;
  }
  // node_status
  if (!dhtt_msgs__msg__NodeStatus__copy(
      &(input->node_status), &(output->node_status)))
  {
    return false;
  }
  return true;
}

dhtt_msgs__msg__Node *
dhtt_msgs__msg__Node__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dhtt_msgs__msg__Node * msg = (dhtt_msgs__msg__Node *)allocator.allocate(sizeof(dhtt_msgs__msg__Node), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dhtt_msgs__msg__Node));
  bool success = dhtt_msgs__msg__Node__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dhtt_msgs__msg__Node__destroy(dhtt_msgs__msg__Node * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dhtt_msgs__msg__Node__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dhtt_msgs__msg__Node__Sequence__init(dhtt_msgs__msg__Node__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dhtt_msgs__msg__Node * data = NULL;

  if (size) {
    data = (dhtt_msgs__msg__Node *)allocator.zero_allocate(size, sizeof(dhtt_msgs__msg__Node), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dhtt_msgs__msg__Node__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dhtt_msgs__msg__Node__fini(&data[i - 1]);
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
dhtt_msgs__msg__Node__Sequence__fini(dhtt_msgs__msg__Node__Sequence * array)
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
      dhtt_msgs__msg__Node__fini(&array->data[i]);
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

dhtt_msgs__msg__Node__Sequence *
dhtt_msgs__msg__Node__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dhtt_msgs__msg__Node__Sequence * array = (dhtt_msgs__msg__Node__Sequence *)allocator.allocate(sizeof(dhtt_msgs__msg__Node__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dhtt_msgs__msg__Node__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dhtt_msgs__msg__Node__Sequence__destroy(dhtt_msgs__msg__Node__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dhtt_msgs__msg__Node__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dhtt_msgs__msg__Node__Sequence__are_equal(const dhtt_msgs__msg__Node__Sequence * lhs, const dhtt_msgs__msg__Node__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dhtt_msgs__msg__Node__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dhtt_msgs__msg__Node__Sequence__copy(
  const dhtt_msgs__msg__Node__Sequence * input,
  dhtt_msgs__msg__Node__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dhtt_msgs__msg__Node);
    dhtt_msgs__msg__Node * data =
      (dhtt_msgs__msg__Node *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dhtt_msgs__msg__Node__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          dhtt_msgs__msg__Node__fini(&data[i]);
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
    if (!dhtt_msgs__msg__Node__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
