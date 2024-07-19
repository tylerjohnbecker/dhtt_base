// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dhtt_msgs:action/Activation.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__ACTION__DETAIL__ACTIVATION__STRUCT_H_
#define DHTT_MSGS__ACTION__DETAIL__ACTIVATION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'passed_resources'
// Member 'granted_resources'
#include "dhtt_msgs/msg/detail/resource__struct.h"

// Struct defined in action/Activation in the package dhtt_msgs.
typedef struct dhtt_msgs__action__Activation_Goal
{
  dhtt_msgs__msg__Resource__Sequence passed_resources;
  dhtt_msgs__msg__Resource__Sequence granted_resources;
  bool success;
} dhtt_msgs__action__Activation_Goal;

// Struct for a sequence of dhtt_msgs__action__Activation_Goal.
typedef struct dhtt_msgs__action__Activation_Goal__Sequence
{
  dhtt_msgs__action__Activation_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dhtt_msgs__action__Activation_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'local_best_node'
#include "rosidl_runtime_c/string.h"
// Member 'requested_resources'
// Member 'owned_resources'
// Member 'released_resources'
// Member 'passed_resources'
// already included above
// #include "dhtt_msgs/msg/detail/resource__struct.h"

// Struct defined in action/Activation in the package dhtt_msgs.
typedef struct dhtt_msgs__action__Activation_Result
{
  rosidl_runtime_c__String local_best_node;
  dhtt_msgs__msg__Resource__Sequence requested_resources;
  dhtt_msgs__msg__Resource__Sequence owned_resources;
  bool done;
  bool possible;
  double activation_potential;
  dhtt_msgs__msg__Resource__Sequence released_resources;
  dhtt_msgs__msg__Resource__Sequence passed_resources;
} dhtt_msgs__action__Activation_Result;

// Struct for a sequence of dhtt_msgs__action__Activation_Result.
typedef struct dhtt_msgs__action__Activation_Result__Sequence
{
  dhtt_msgs__action__Activation_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dhtt_msgs__action__Activation_Result__Sequence;


// Constants defined in the message

// Struct defined in action/Activation in the package dhtt_msgs.
typedef struct dhtt_msgs__action__Activation_Feedback
{
  uint8_t structure_needs_at_least_one_member;
} dhtt_msgs__action__Activation_Feedback;

// Struct for a sequence of dhtt_msgs__action__Activation_Feedback.
typedef struct dhtt_msgs__action__Activation_Feedback__Sequence
{
  dhtt_msgs__action__Activation_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dhtt_msgs__action__Activation_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "dhtt_msgs/action/detail/activation__struct.h"

// Struct defined in action/Activation in the package dhtt_msgs.
typedef struct dhtt_msgs__action__Activation_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  dhtt_msgs__action__Activation_Goal goal;
} dhtt_msgs__action__Activation_SendGoal_Request;

// Struct for a sequence of dhtt_msgs__action__Activation_SendGoal_Request.
typedef struct dhtt_msgs__action__Activation_SendGoal_Request__Sequence
{
  dhtt_msgs__action__Activation_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dhtt_msgs__action__Activation_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

// Struct defined in action/Activation in the package dhtt_msgs.
typedef struct dhtt_msgs__action__Activation_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} dhtt_msgs__action__Activation_SendGoal_Response;

// Struct for a sequence of dhtt_msgs__action__Activation_SendGoal_Response.
typedef struct dhtt_msgs__action__Activation_SendGoal_Response__Sequence
{
  dhtt_msgs__action__Activation_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dhtt_msgs__action__Activation_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

// Struct defined in action/Activation in the package dhtt_msgs.
typedef struct dhtt_msgs__action__Activation_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} dhtt_msgs__action__Activation_GetResult_Request;

// Struct for a sequence of dhtt_msgs__action__Activation_GetResult_Request.
typedef struct dhtt_msgs__action__Activation_GetResult_Request__Sequence
{
  dhtt_msgs__action__Activation_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dhtt_msgs__action__Activation_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "dhtt_msgs/action/detail/activation__struct.h"

// Struct defined in action/Activation in the package dhtt_msgs.
typedef struct dhtt_msgs__action__Activation_GetResult_Response
{
  int8_t status;
  dhtt_msgs__action__Activation_Result result;
} dhtt_msgs__action__Activation_GetResult_Response;

// Struct for a sequence of dhtt_msgs__action__Activation_GetResult_Response.
typedef struct dhtt_msgs__action__Activation_GetResult_Response__Sequence
{
  dhtt_msgs__action__Activation_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dhtt_msgs__action__Activation_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "dhtt_msgs/action/detail/activation__struct.h"

// Struct defined in action/Activation in the package dhtt_msgs.
typedef struct dhtt_msgs__action__Activation_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  dhtt_msgs__action__Activation_Feedback feedback;
} dhtt_msgs__action__Activation_FeedbackMessage;

// Struct for a sequence of dhtt_msgs__action__Activation_FeedbackMessage.
typedef struct dhtt_msgs__action__Activation_FeedbackMessage__Sequence
{
  dhtt_msgs__action__Activation_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dhtt_msgs__action__Activation_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DHTT_MSGS__ACTION__DETAIL__ACTIVATION__STRUCT_H_
