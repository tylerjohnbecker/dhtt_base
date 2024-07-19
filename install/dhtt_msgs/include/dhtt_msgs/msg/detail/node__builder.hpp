// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dhtt_msgs:msg/Node.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__MSG__DETAIL__NODE__BUILDER_HPP_
#define DHTT_MSGS__MSG__DETAIL__NODE__BUILDER_HPP_

#include "dhtt_msgs/msg/detail/node__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace dhtt_msgs
{

namespace msg
{

namespace builder
{

class Init_Node_node_status
{
public:
  explicit Init_Node_node_status(::dhtt_msgs::msg::Node & msg)
  : msg_(msg)
  {}
  ::dhtt_msgs::msg::Node node_status(::dhtt_msgs::msg::Node::_node_status_type arg)
  {
    msg_.node_status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dhtt_msgs::msg::Node msg_;
};

class Init_Node_owned_resources
{
public:
  explicit Init_Node_owned_resources(::dhtt_msgs::msg::Node & msg)
  : msg_(msg)
  {}
  Init_Node_node_status owned_resources(::dhtt_msgs::msg::Node::_owned_resources_type arg)
  {
    msg_.owned_resources = std::move(arg);
    return Init_Node_node_status(msg_);
  }

private:
  ::dhtt_msgs::msg::Node msg_;
};

class Init_Node_plugin_name
{
public:
  explicit Init_Node_plugin_name(::dhtt_msgs::msg::Node & msg)
  : msg_(msg)
  {}
  Init_Node_owned_resources plugin_name(::dhtt_msgs::msg::Node::_plugin_name_type arg)
  {
    msg_.plugin_name = std::move(arg);
    return Init_Node_owned_resources(msg_);
  }

private:
  ::dhtt_msgs::msg::Node msg_;
};

class Init_Node_type
{
public:
  explicit Init_Node_type(::dhtt_msgs::msg::Node & msg)
  : msg_(msg)
  {}
  Init_Node_plugin_name type(::dhtt_msgs::msg::Node::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_Node_plugin_name(msg_);
  }

private:
  ::dhtt_msgs::msg::Node msg_;
};

class Init_Node_params
{
public:
  explicit Init_Node_params(::dhtt_msgs::msg::Node & msg)
  : msg_(msg)
  {}
  Init_Node_type params(::dhtt_msgs::msg::Node::_params_type arg)
  {
    msg_.params = std::move(arg);
    return Init_Node_type(msg_);
  }

private:
  ::dhtt_msgs::msg::Node msg_;
};

class Init_Node_child_name
{
public:
  explicit Init_Node_child_name(::dhtt_msgs::msg::Node & msg)
  : msg_(msg)
  {}
  Init_Node_params child_name(::dhtt_msgs::msg::Node::_child_name_type arg)
  {
    msg_.child_name = std::move(arg);
    return Init_Node_params(msg_);
  }

private:
  ::dhtt_msgs::msg::Node msg_;
};

class Init_Node_children
{
public:
  explicit Init_Node_children(::dhtt_msgs::msg::Node & msg)
  : msg_(msg)
  {}
  Init_Node_child_name children(::dhtt_msgs::msg::Node::_children_type arg)
  {
    msg_.children = std::move(arg);
    return Init_Node_child_name(msg_);
  }

private:
  ::dhtt_msgs::msg::Node msg_;
};

class Init_Node_parent_name
{
public:
  explicit Init_Node_parent_name(::dhtt_msgs::msg::Node & msg)
  : msg_(msg)
  {}
  Init_Node_children parent_name(::dhtt_msgs::msg::Node::_parent_name_type arg)
  {
    msg_.parent_name = std::move(arg);
    return Init_Node_children(msg_);
  }

private:
  ::dhtt_msgs::msg::Node msg_;
};

class Init_Node_parent
{
public:
  explicit Init_Node_parent(::dhtt_msgs::msg::Node & msg)
  : msg_(msg)
  {}
  Init_Node_parent_name parent(::dhtt_msgs::msg::Node::_parent_type arg)
  {
    msg_.parent = std::move(arg);
    return Init_Node_parent_name(msg_);
  }

private:
  ::dhtt_msgs::msg::Node msg_;
};

class Init_Node_node_name
{
public:
  explicit Init_Node_node_name(::dhtt_msgs::msg::Node & msg)
  : msg_(msg)
  {}
  Init_Node_parent node_name(::dhtt_msgs::msg::Node::_node_name_type arg)
  {
    msg_.node_name = std::move(arg);
    return Init_Node_parent(msg_);
  }

private:
  ::dhtt_msgs::msg::Node msg_;
};

class Init_Node_head
{
public:
  Init_Node_head()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Node_node_name head(::dhtt_msgs::msg::Node::_head_type arg)
  {
    msg_.head = std::move(arg);
    return Init_Node_node_name(msg_);
  }

private:
  ::dhtt_msgs::msg::Node msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dhtt_msgs::msg::Node>()
{
  return dhtt_msgs::msg::builder::Init_Node_head();
}

}  // namespace dhtt_msgs

#endif  // DHTT_MSGS__MSG__DETAIL__NODE__BUILDER_HPP_
