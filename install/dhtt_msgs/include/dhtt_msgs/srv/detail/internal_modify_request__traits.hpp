// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dhtt_msgs:srv/InternalModifyRequest.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__SRV__DETAIL__INTERNAL_MODIFY_REQUEST__TRAITS_HPP_
#define DHTT_MSGS__SRV__DETAIL__INTERNAL_MODIFY_REQUEST__TRAITS_HPP_

#include "dhtt_msgs/srv/detail/internal_modify_request__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<dhtt_msgs::srv::InternalModifyRequest_Request>()
{
  return "dhtt_msgs::srv::InternalModifyRequest_Request";
}

template<>
inline const char * name<dhtt_msgs::srv::InternalModifyRequest_Request>()
{
  return "dhtt_msgs/srv/InternalModifyRequest_Request";
}

template<>
struct has_fixed_size<dhtt_msgs::srv::InternalModifyRequest_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dhtt_msgs::srv::InternalModifyRequest_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dhtt_msgs::srv::InternalModifyRequest_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<dhtt_msgs::srv::InternalModifyRequest_Response>()
{
  return "dhtt_msgs::srv::InternalModifyRequest_Response";
}

template<>
inline const char * name<dhtt_msgs::srv::InternalModifyRequest_Response>()
{
  return "dhtt_msgs/srv/InternalModifyRequest_Response";
}

template<>
struct has_fixed_size<dhtt_msgs::srv::InternalModifyRequest_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dhtt_msgs::srv::InternalModifyRequest_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dhtt_msgs::srv::InternalModifyRequest_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<dhtt_msgs::srv::InternalModifyRequest>()
{
  return "dhtt_msgs::srv::InternalModifyRequest";
}

template<>
inline const char * name<dhtt_msgs::srv::InternalModifyRequest>()
{
  return "dhtt_msgs/srv/InternalModifyRequest";
}

template<>
struct has_fixed_size<dhtt_msgs::srv::InternalModifyRequest>
  : std::integral_constant<
    bool,
    has_fixed_size<dhtt_msgs::srv::InternalModifyRequest_Request>::value &&
    has_fixed_size<dhtt_msgs::srv::InternalModifyRequest_Response>::value
  >
{
};

template<>
struct has_bounded_size<dhtt_msgs::srv::InternalModifyRequest>
  : std::integral_constant<
    bool,
    has_bounded_size<dhtt_msgs::srv::InternalModifyRequest_Request>::value &&
    has_bounded_size<dhtt_msgs::srv::InternalModifyRequest_Response>::value
  >
{
};

template<>
struct is_service<dhtt_msgs::srv::InternalModifyRequest>
  : std::true_type
{
};

template<>
struct is_service_request<dhtt_msgs::srv::InternalModifyRequest_Request>
  : std::true_type
{
};

template<>
struct is_service_response<dhtt_msgs::srv::InternalModifyRequest_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // DHTT_MSGS__SRV__DETAIL__INTERNAL_MODIFY_REQUEST__TRAITS_HPP_
