// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dhtt_msgs:msg/NodeStatus.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__MSG__DETAIL__NODE_STATUS__TRAITS_HPP_
#define DHTT_MSGS__MSG__DETAIL__NODE_STATUS__TRAITS_HPP_

#include "dhtt_msgs/msg/detail/node_status__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<dhtt_msgs::msg::NodeStatus>()
{
  return "dhtt_msgs::msg::NodeStatus";
}

template<>
inline const char * name<dhtt_msgs::msg::NodeStatus>()
{
  return "dhtt_msgs/msg/NodeStatus";
}

template<>
struct has_fixed_size<dhtt_msgs::msg::NodeStatus>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<dhtt_msgs::msg::NodeStatus>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<dhtt_msgs::msg::NodeStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DHTT_MSGS__MSG__DETAIL__NODE_STATUS__TRAITS_HPP_
