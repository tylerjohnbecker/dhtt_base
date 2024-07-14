// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from knowledge_server:srv/Fetch.idl
// generated code does not contain a copyright notice

#ifndef KNOWLEDGE_SERVER__SRV__DETAIL__FETCH__TRAITS_HPP_
#define KNOWLEDGE_SERVER__SRV__DETAIL__FETCH__TRAITS_HPP_

#include "knowledge_server/srv/detail/fetch__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<knowledge_server::srv::Fetch_Request>()
{
  return "knowledge_server::srv::Fetch_Request";
}

template<>
inline const char * name<knowledge_server::srv::Fetch_Request>()
{
  return "knowledge_server/srv/Fetch_Request";
}

template<>
struct has_fixed_size<knowledge_server::srv::Fetch_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<knowledge_server::srv::Fetch_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<knowledge_server::srv::Fetch_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<knowledge_server::srv::Fetch_Response>()
{
  return "knowledge_server::srv::Fetch_Response";
}

template<>
inline const char * name<knowledge_server::srv::Fetch_Response>()
{
  return "knowledge_server/srv/Fetch_Response";
}

template<>
struct has_fixed_size<knowledge_server::srv::Fetch_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<knowledge_server::srv::Fetch_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<knowledge_server::srv::Fetch_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<knowledge_server::srv::Fetch>()
{
  return "knowledge_server::srv::Fetch";
}

template<>
inline const char * name<knowledge_server::srv::Fetch>()
{
  return "knowledge_server/srv/Fetch";
}

template<>
struct has_fixed_size<knowledge_server::srv::Fetch>
  : std::integral_constant<
    bool,
    has_fixed_size<knowledge_server::srv::Fetch_Request>::value &&
    has_fixed_size<knowledge_server::srv::Fetch_Response>::value
  >
{
};

template<>
struct has_bounded_size<knowledge_server::srv::Fetch>
  : std::integral_constant<
    bool,
    has_bounded_size<knowledge_server::srv::Fetch_Request>::value &&
    has_bounded_size<knowledge_server::srv::Fetch_Response>::value
  >
{
};

template<>
struct is_service<knowledge_server::srv::Fetch>
  : std::true_type
{
};

template<>
struct is_service_request<knowledge_server::srv::Fetch_Request>
  : std::true_type
{
};

template<>
struct is_service_response<knowledge_server::srv::Fetch_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // KNOWLEDGE_SERVER__SRV__DETAIL__FETCH__TRAITS_HPP_
