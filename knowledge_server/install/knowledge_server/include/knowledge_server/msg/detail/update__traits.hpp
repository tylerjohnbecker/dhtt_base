// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from knowledge_server:msg/Update.idl
// generated code does not contain a copyright notice

#ifndef KNOWLEDGE_SERVER__MSG__DETAIL__UPDATE__TRAITS_HPP_
#define KNOWLEDGE_SERVER__MSG__DETAIL__UPDATE__TRAITS_HPP_

#include "knowledge_server/msg/detail/update__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<knowledge_server::msg::Update>()
{
  return "knowledge_server::msg::Update";
}

template<>
inline const char * name<knowledge_server::msg::Update>()
{
  return "knowledge_server/msg/Update";
}

template<>
struct has_fixed_size<knowledge_server::msg::Update>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<knowledge_server::msg::Update>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<knowledge_server::msg::Update>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // KNOWLEDGE_SERVER__MSG__DETAIL__UPDATE__TRAITS_HPP_
