// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dhtt_msgs:srv/FetchRequest.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__SRV__DETAIL__FETCH_REQUEST__BUILDER_HPP_
#define DHTT_MSGS__SRV__DETAIL__FETCH_REQUEST__BUILDER_HPP_

#include "dhtt_msgs/srv/detail/fetch_request__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace dhtt_msgs
{

namespace srv
{

namespace builder
{

class Init_FetchRequest_Request_node_type
{
public:
  explicit Init_FetchRequest_Request_node_type(::dhtt_msgs::srv::FetchRequest_Request & msg)
  : msg_(msg)
  {}
  ::dhtt_msgs::srv::FetchRequest_Request node_type(::dhtt_msgs::srv::FetchRequest_Request::_node_type_type arg)
  {
    msg_.node_type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dhtt_msgs::srv::FetchRequest_Request msg_;
};

class Init_FetchRequest_Request_node_name
{
public:
  explicit Init_FetchRequest_Request_node_name(::dhtt_msgs::srv::FetchRequest_Request & msg)
  : msg_(msg)
  {}
  Init_FetchRequest_Request_node_type node_name(::dhtt_msgs::srv::FetchRequest_Request::_node_name_type arg)
  {
    msg_.node_name = std::move(arg);
    return Init_FetchRequest_Request_node_type(msg_);
  }

private:
  ::dhtt_msgs::srv::FetchRequest_Request msg_;
};

class Init_FetchRequest_Request_common_name
{
public:
  explicit Init_FetchRequest_Request_common_name(::dhtt_msgs::srv::FetchRequest_Request & msg)
  : msg_(msg)
  {}
  Init_FetchRequest_Request_node_name common_name(::dhtt_msgs::srv::FetchRequest_Request::_common_name_type arg)
  {
    msg_.common_name = std::move(arg);
    return Init_FetchRequest_Request_node_name(msg_);
  }

private:
  ::dhtt_msgs::srv::FetchRequest_Request msg_;
};

class Init_FetchRequest_Request_return_full_subtree
{
public:
  Init_FetchRequest_Request_return_full_subtree()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FetchRequest_Request_common_name return_full_subtree(::dhtt_msgs::srv::FetchRequest_Request::_return_full_subtree_type arg)
  {
    msg_.return_full_subtree = std::move(arg);
    return Init_FetchRequest_Request_common_name(msg_);
  }

private:
  ::dhtt_msgs::srv::FetchRequest_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dhtt_msgs::srv::FetchRequest_Request>()
{
  return dhtt_msgs::srv::builder::Init_FetchRequest_Request_return_full_subtree();
}

}  // namespace dhtt_msgs


namespace dhtt_msgs
{

namespace srv
{

namespace builder
{

class Init_FetchRequest_Response_found_subtrees
{
public:
  explicit Init_FetchRequest_Response_found_subtrees(::dhtt_msgs::srv::FetchRequest_Response & msg)
  : msg_(msg)
  {}
  ::dhtt_msgs::srv::FetchRequest_Response found_subtrees(::dhtt_msgs::srv::FetchRequest_Response::_found_subtrees_type arg)
  {
    msg_.found_subtrees = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dhtt_msgs::srv::FetchRequest_Response msg_;
};

class Init_FetchRequest_Response_error_msg
{
public:
  explicit Init_FetchRequest_Response_error_msg(::dhtt_msgs::srv::FetchRequest_Response & msg)
  : msg_(msg)
  {}
  Init_FetchRequest_Response_found_subtrees error_msg(::dhtt_msgs::srv::FetchRequest_Response::_error_msg_type arg)
  {
    msg_.error_msg = std::move(arg);
    return Init_FetchRequest_Response_found_subtrees(msg_);
  }

private:
  ::dhtt_msgs::srv::FetchRequest_Response msg_;
};

class Init_FetchRequest_Response_success
{
public:
  Init_FetchRequest_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FetchRequest_Response_error_msg success(::dhtt_msgs::srv::FetchRequest_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_FetchRequest_Response_error_msg(msg_);
  }

private:
  ::dhtt_msgs::srv::FetchRequest_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dhtt_msgs::srv::FetchRequest_Response>()
{
  return dhtt_msgs::srv::builder::Init_FetchRequest_Response_success();
}

}  // namespace dhtt_msgs

#endif  // DHTT_MSGS__SRV__DETAIL__FETCH_REQUEST__BUILDER_HPP_
