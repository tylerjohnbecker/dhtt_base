// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dhtt_msgs:srv/InternalServiceRegistration.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__SRV__DETAIL__INTERNAL_SERVICE_REGISTRATION__BUILDER_HPP_
#define DHTT_MSGS__SRV__DETAIL__INTERNAL_SERVICE_REGISTRATION__BUILDER_HPP_

#include "dhtt_msgs/srv/detail/internal_service_registration__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace dhtt_msgs
{

namespace srv
{

namespace builder
{

class Init_InternalServiceRegistration_Request_node_name
{
public:
  Init_InternalServiceRegistration_Request_node_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dhtt_msgs::srv::InternalServiceRegistration_Request node_name(::dhtt_msgs::srv::InternalServiceRegistration_Request::_node_name_type arg)
  {
    msg_.node_name = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dhtt_msgs::srv::InternalServiceRegistration_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dhtt_msgs::srv::InternalServiceRegistration_Request>()
{
  return dhtt_msgs::srv::builder::Init_InternalServiceRegistration_Request_node_name();
}

}  // namespace dhtt_msgs


namespace dhtt_msgs
{

namespace srv
{

namespace builder
{

class Init_InternalServiceRegistration_Response_success
{
public:
  Init_InternalServiceRegistration_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dhtt_msgs::srv::InternalServiceRegistration_Response success(::dhtt_msgs::srv::InternalServiceRegistration_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dhtt_msgs::srv::InternalServiceRegistration_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dhtt_msgs::srv::InternalServiceRegistration_Response>()
{
  return dhtt_msgs::srv::builder::Init_InternalServiceRegistration_Response_success();
}

}  // namespace dhtt_msgs

#endif  // DHTT_MSGS__SRV__DETAIL__INTERNAL_SERVICE_REGISTRATION__BUILDER_HPP_
