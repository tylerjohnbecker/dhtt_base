// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dhtt_msgs:srv/ModifyRequest.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__SRV__DETAIL__MODIFY_REQUEST__TRAITS_HPP_
#define DHTT_MSGS__SRV__DETAIL__MODIFY_REQUEST__TRAITS_HPP_

#include "dhtt_msgs/srv/detail/modify_request__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'add_node'
#include "dhtt_msgs/msg/detail/node__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<dhtt_msgs::srv::ModifyRequest_Request>()
{
  return "dhtt_msgs::srv::ModifyRequest_Request";
}

template<>
inline const char * name<dhtt_msgs::srv::ModifyRequest_Request>()
{
  return "dhtt_msgs/srv/ModifyRequest_Request";
}

template<>
struct has_fixed_size<dhtt_msgs::srv::ModifyRequest_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dhtt_msgs::srv::ModifyRequest_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dhtt_msgs::srv::ModifyRequest_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<dhtt_msgs::srv::ModifyRequest_Response>()
{
  return "dhtt_msgs::srv::ModifyRequest_Response";
}

template<>
inline const char * name<dhtt_msgs::srv::ModifyRequest_Response>()
{
  return "dhtt_msgs/srv/ModifyRequest_Response";
}

template<>
struct has_fixed_size<dhtt_msgs::srv::ModifyRequest_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dhtt_msgs::srv::ModifyRequest_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dhtt_msgs::srv::ModifyRequest_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<dhtt_msgs::srv::ModifyRequest>()
{
  return "dhtt_msgs::srv::ModifyRequest";
}

template<>
inline const char * name<dhtt_msgs::srv::ModifyRequest>()
{
  return "dhtt_msgs/srv/ModifyRequest";
}

template<>
struct has_fixed_size<dhtt_msgs::srv::ModifyRequest>
  : std::integral_constant<
    bool,
    has_fixed_size<dhtt_msgs::srv::ModifyRequest_Request>::value &&
    has_fixed_size<dhtt_msgs::srv::ModifyRequest_Response>::value
  >
{
};

template<>
struct has_bounded_size<dhtt_msgs::srv::ModifyRequest>
  : std::integral_constant<
    bool,
    has_bounded_size<dhtt_msgs::srv::ModifyRequest_Request>::value &&
    has_bounded_size<dhtt_msgs::srv::ModifyRequest_Response>::value
  >
{
};

template<>
struct is_service<dhtt_msgs::srv::ModifyRequest>
  : std::true_type
{
};

template<>
struct is_service_request<dhtt_msgs::srv::ModifyRequest_Request>
  : std::true_type
{
};

template<>
struct is_service_response<dhtt_msgs::srv::ModifyRequest_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // DHTT_MSGS__SRV__DETAIL__MODIFY_REQUEST__TRAITS_HPP_
