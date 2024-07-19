// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dhtt_msgs:srv/ModifyRequest.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__SRV__DETAIL__MODIFY_REQUEST__BUILDER_HPP_
#define DHTT_MSGS__SRV__DETAIL__MODIFY_REQUEST__BUILDER_HPP_

#include "dhtt_msgs/srv/detail/modify_request__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace dhtt_msgs
{

namespace srv
{

namespace builder
{

class Init_ModifyRequest_Request_mutate_type
{
public:
  explicit Init_ModifyRequest_Request_mutate_type(::dhtt_msgs::srv::ModifyRequest_Request & msg)
  : msg_(msg)
  {}
  ::dhtt_msgs::srv::ModifyRequest_Request mutate_type(::dhtt_msgs::srv::ModifyRequest_Request::_mutate_type_type arg)
  {
    msg_.mutate_type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dhtt_msgs::srv::ModifyRequest_Request msg_;
};

class Init_ModifyRequest_Request_params
{
public:
  explicit Init_ModifyRequest_Request_params(::dhtt_msgs::srv::ModifyRequest_Request & msg)
  : msg_(msg)
  {}
  Init_ModifyRequest_Request_mutate_type params(::dhtt_msgs::srv::ModifyRequest_Request::_params_type arg)
  {
    msg_.params = std::move(arg);
    return Init_ModifyRequest_Request_mutate_type(msg_);
  }

private:
  ::dhtt_msgs::srv::ModifyRequest_Request msg_;
};

class Init_ModifyRequest_Request_to_add
{
public:
  explicit Init_ModifyRequest_Request_to_add(::dhtt_msgs::srv::ModifyRequest_Request & msg)
  : msg_(msg)
  {}
  Init_ModifyRequest_Request_params to_add(::dhtt_msgs::srv::ModifyRequest_Request::_to_add_type arg)
  {
    msg_.to_add = std::move(arg);
    return Init_ModifyRequest_Request_params(msg_);
  }

private:
  ::dhtt_msgs::srv::ModifyRequest_Request msg_;
};

class Init_ModifyRequest_Request_add_node
{
public:
  explicit Init_ModifyRequest_Request_add_node(::dhtt_msgs::srv::ModifyRequest_Request & msg)
  : msg_(msg)
  {}
  Init_ModifyRequest_Request_to_add add_node(::dhtt_msgs::srv::ModifyRequest_Request::_add_node_type arg)
  {
    msg_.add_node = std::move(arg);
    return Init_ModifyRequest_Request_to_add(msg_);
  }

private:
  ::dhtt_msgs::srv::ModifyRequest_Request msg_;
};

class Init_ModifyRequest_Request_to_modify
{
public:
  explicit Init_ModifyRequest_Request_to_modify(::dhtt_msgs::srv::ModifyRequest_Request & msg)
  : msg_(msg)
  {}
  Init_ModifyRequest_Request_add_node to_modify(::dhtt_msgs::srv::ModifyRequest_Request::_to_modify_type arg)
  {
    msg_.to_modify = std::move(arg);
    return Init_ModifyRequest_Request_add_node(msg_);
  }

private:
  ::dhtt_msgs::srv::ModifyRequest_Request msg_;
};

class Init_ModifyRequest_Request_type
{
public:
  Init_ModifyRequest_Request_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ModifyRequest_Request_to_modify type(::dhtt_msgs::srv::ModifyRequest_Request::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_ModifyRequest_Request_to_modify(msg_);
  }

private:
  ::dhtt_msgs::srv::ModifyRequest_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dhtt_msgs::srv::ModifyRequest_Request>()
{
  return dhtt_msgs::srv::builder::Init_ModifyRequest_Request_type();
}

}  // namespace dhtt_msgs


namespace dhtt_msgs
{

namespace srv
{

namespace builder
{

class Init_ModifyRequest_Response_removed_nodes
{
public:
  explicit Init_ModifyRequest_Response_removed_nodes(::dhtt_msgs::srv::ModifyRequest_Response & msg)
  : msg_(msg)
  {}
  ::dhtt_msgs::srv::ModifyRequest_Response removed_nodes(::dhtt_msgs::srv::ModifyRequest_Response::_removed_nodes_type arg)
  {
    msg_.removed_nodes = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dhtt_msgs::srv::ModifyRequest_Response msg_;
};

class Init_ModifyRequest_Response_added_nodes
{
public:
  explicit Init_ModifyRequest_Response_added_nodes(::dhtt_msgs::srv::ModifyRequest_Response & msg)
  : msg_(msg)
  {}
  Init_ModifyRequest_Response_removed_nodes added_nodes(::dhtt_msgs::srv::ModifyRequest_Response::_added_nodes_type arg)
  {
    msg_.added_nodes = std::move(arg);
    return Init_ModifyRequest_Response_removed_nodes(msg_);
  }

private:
  ::dhtt_msgs::srv::ModifyRequest_Response msg_;
};

class Init_ModifyRequest_Response_error_msg
{
public:
  explicit Init_ModifyRequest_Response_error_msg(::dhtt_msgs::srv::ModifyRequest_Response & msg)
  : msg_(msg)
  {}
  Init_ModifyRequest_Response_added_nodes error_msg(::dhtt_msgs::srv::ModifyRequest_Response::_error_msg_type arg)
  {
    msg_.error_msg = std::move(arg);
    return Init_ModifyRequest_Response_added_nodes(msg_);
  }

private:
  ::dhtt_msgs::srv::ModifyRequest_Response msg_;
};

class Init_ModifyRequest_Response_success
{
public:
  Init_ModifyRequest_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ModifyRequest_Response_error_msg success(::dhtt_msgs::srv::ModifyRequest_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_ModifyRequest_Response_error_msg(msg_);
  }

private:
  ::dhtt_msgs::srv::ModifyRequest_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dhtt_msgs::srv::ModifyRequest_Response>()
{
  return dhtt_msgs::srv::builder::Init_ModifyRequest_Response_success();
}

}  // namespace dhtt_msgs

#endif  // DHTT_MSGS__SRV__DETAIL__MODIFY_REQUEST__BUILDER_HPP_
