// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dhtt_msgs:action/Activation.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__ACTION__DETAIL__ACTIVATION__TRAITS_HPP_
#define DHTT_MSGS__ACTION__DETAIL__ACTIVATION__TRAITS_HPP_

#include "dhtt_msgs/action/detail/activation__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<dhtt_msgs::action::Activation_Goal>()
{
  return "dhtt_msgs::action::Activation_Goal";
}

template<>
inline const char * name<dhtt_msgs::action::Activation_Goal>()
{
  return "dhtt_msgs/action/Activation_Goal";
}

template<>
struct has_fixed_size<dhtt_msgs::action::Activation_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dhtt_msgs::action::Activation_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dhtt_msgs::action::Activation_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<dhtt_msgs::action::Activation_Result>()
{
  return "dhtt_msgs::action::Activation_Result";
}

template<>
inline const char * name<dhtt_msgs::action::Activation_Result>()
{
  return "dhtt_msgs/action/Activation_Result";
}

template<>
struct has_fixed_size<dhtt_msgs::action::Activation_Result>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dhtt_msgs::action::Activation_Result>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dhtt_msgs::action::Activation_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<dhtt_msgs::action::Activation_Feedback>()
{
  return "dhtt_msgs::action::Activation_Feedback";
}

template<>
inline const char * name<dhtt_msgs::action::Activation_Feedback>()
{
  return "dhtt_msgs/action/Activation_Feedback";
}

template<>
struct has_fixed_size<dhtt_msgs::action::Activation_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<dhtt_msgs::action::Activation_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<dhtt_msgs::action::Activation_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "dhtt_msgs/action/detail/activation__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<dhtt_msgs::action::Activation_SendGoal_Request>()
{
  return "dhtt_msgs::action::Activation_SendGoal_Request";
}

template<>
inline const char * name<dhtt_msgs::action::Activation_SendGoal_Request>()
{
  return "dhtt_msgs/action/Activation_SendGoal_Request";
}

template<>
struct has_fixed_size<dhtt_msgs::action::Activation_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<dhtt_msgs::action::Activation_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<dhtt_msgs::action::Activation_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<dhtt_msgs::action::Activation_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<dhtt_msgs::action::Activation_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<dhtt_msgs::action::Activation_SendGoal_Response>()
{
  return "dhtt_msgs::action::Activation_SendGoal_Response";
}

template<>
inline const char * name<dhtt_msgs::action::Activation_SendGoal_Response>()
{
  return "dhtt_msgs/action/Activation_SendGoal_Response";
}

template<>
struct has_fixed_size<dhtt_msgs::action::Activation_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<dhtt_msgs::action::Activation_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<dhtt_msgs::action::Activation_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<dhtt_msgs::action::Activation_SendGoal>()
{
  return "dhtt_msgs::action::Activation_SendGoal";
}

template<>
inline const char * name<dhtt_msgs::action::Activation_SendGoal>()
{
  return "dhtt_msgs/action/Activation_SendGoal";
}

template<>
struct has_fixed_size<dhtt_msgs::action::Activation_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<dhtt_msgs::action::Activation_SendGoal_Request>::value &&
    has_fixed_size<dhtt_msgs::action::Activation_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<dhtt_msgs::action::Activation_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<dhtt_msgs::action::Activation_SendGoal_Request>::value &&
    has_bounded_size<dhtt_msgs::action::Activation_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<dhtt_msgs::action::Activation_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<dhtt_msgs::action::Activation_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<dhtt_msgs::action::Activation_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<dhtt_msgs::action::Activation_GetResult_Request>()
{
  return "dhtt_msgs::action::Activation_GetResult_Request";
}

template<>
inline const char * name<dhtt_msgs::action::Activation_GetResult_Request>()
{
  return "dhtt_msgs/action/Activation_GetResult_Request";
}

template<>
struct has_fixed_size<dhtt_msgs::action::Activation_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<dhtt_msgs::action::Activation_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<dhtt_msgs::action::Activation_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "dhtt_msgs/action/detail/activation__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<dhtt_msgs::action::Activation_GetResult_Response>()
{
  return "dhtt_msgs::action::Activation_GetResult_Response";
}

template<>
inline const char * name<dhtt_msgs::action::Activation_GetResult_Response>()
{
  return "dhtt_msgs/action/Activation_GetResult_Response";
}

template<>
struct has_fixed_size<dhtt_msgs::action::Activation_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<dhtt_msgs::action::Activation_Result>::value> {};

template<>
struct has_bounded_size<dhtt_msgs::action::Activation_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<dhtt_msgs::action::Activation_Result>::value> {};

template<>
struct is_message<dhtt_msgs::action::Activation_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<dhtt_msgs::action::Activation_GetResult>()
{
  return "dhtt_msgs::action::Activation_GetResult";
}

template<>
inline const char * name<dhtt_msgs::action::Activation_GetResult>()
{
  return "dhtt_msgs/action/Activation_GetResult";
}

template<>
struct has_fixed_size<dhtt_msgs::action::Activation_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<dhtt_msgs::action::Activation_GetResult_Request>::value &&
    has_fixed_size<dhtt_msgs::action::Activation_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<dhtt_msgs::action::Activation_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<dhtt_msgs::action::Activation_GetResult_Request>::value &&
    has_bounded_size<dhtt_msgs::action::Activation_GetResult_Response>::value
  >
{
};

template<>
struct is_service<dhtt_msgs::action::Activation_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<dhtt_msgs::action::Activation_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<dhtt_msgs::action::Activation_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "dhtt_msgs/action/detail/activation__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<dhtt_msgs::action::Activation_FeedbackMessage>()
{
  return "dhtt_msgs::action::Activation_FeedbackMessage";
}

template<>
inline const char * name<dhtt_msgs::action::Activation_FeedbackMessage>()
{
  return "dhtt_msgs/action/Activation_FeedbackMessage";
}

template<>
struct has_fixed_size<dhtt_msgs::action::Activation_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<dhtt_msgs::action::Activation_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<dhtt_msgs::action::Activation_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<dhtt_msgs::action::Activation_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<dhtt_msgs::action::Activation_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<dhtt_msgs::action::Activation>
  : std::true_type
{
};

template<>
struct is_action_goal<dhtt_msgs::action::Activation_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<dhtt_msgs::action::Activation_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<dhtt_msgs::action::Activation_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // DHTT_MSGS__ACTION__DETAIL__ACTIVATION__TRAITS_HPP_
