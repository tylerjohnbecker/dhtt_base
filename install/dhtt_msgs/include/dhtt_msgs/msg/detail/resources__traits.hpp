// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dhtt_msgs:msg/Resources.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__MSG__DETAIL__RESOURCES__TRAITS_HPP_
#define DHTT_MSGS__MSG__DETAIL__RESOURCES__TRAITS_HPP_

#include "dhtt_msgs/msg/detail/resources__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<dhtt_msgs::msg::Resources>()
{
  return "dhtt_msgs::msg::Resources";
}

template<>
inline const char * name<dhtt_msgs::msg::Resources>()
{
  return "dhtt_msgs/msg/Resources";
}

template<>
struct has_fixed_size<dhtt_msgs::msg::Resources>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dhtt_msgs::msg::Resources>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dhtt_msgs::msg::Resources>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DHTT_MSGS__MSG__DETAIL__RESOURCES__TRAITS_HPP_
