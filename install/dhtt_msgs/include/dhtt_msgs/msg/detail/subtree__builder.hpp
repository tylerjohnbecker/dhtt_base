// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dhtt_msgs:msg/Subtree.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__MSG__DETAIL__SUBTREE__BUILDER_HPP_
#define DHTT_MSGS__MSG__DETAIL__SUBTREE__BUILDER_HPP_

#include "dhtt_msgs/msg/detail/subtree__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace dhtt_msgs
{

namespace msg
{

namespace builder
{

class Init_Subtree_task_completion_percent
{
public:
  explicit Init_Subtree_task_completion_percent(::dhtt_msgs::msg::Subtree & msg)
  : msg_(msg)
  {}
  ::dhtt_msgs::msg::Subtree task_completion_percent(::dhtt_msgs::msg::Subtree::_task_completion_percent_type arg)
  {
    msg_.task_completion_percent = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dhtt_msgs::msg::Subtree msg_;
};

class Init_Subtree_max_tree_width
{
public:
  explicit Init_Subtree_max_tree_width(::dhtt_msgs::msg::Subtree & msg)
  : msg_(msg)
  {}
  Init_Subtree_task_completion_percent max_tree_width(::dhtt_msgs::msg::Subtree::_max_tree_width_type arg)
  {
    msg_.max_tree_width = std::move(arg);
    return Init_Subtree_task_completion_percent(msg_);
  }

private:
  ::dhtt_msgs::msg::Subtree msg_;
};

class Init_Subtree_max_tree_depth
{
public:
  explicit Init_Subtree_max_tree_depth(::dhtt_msgs::msg::Subtree & msg)
  : msg_(msg)
  {}
  Init_Subtree_max_tree_width max_tree_depth(::dhtt_msgs::msg::Subtree::_max_tree_depth_type arg)
  {
    msg_.max_tree_depth = std::move(arg);
    return Init_Subtree_max_tree_width(msg_);
  }

private:
  ::dhtt_msgs::msg::Subtree msg_;
};

class Init_Subtree_tree_status
{
public:
  explicit Init_Subtree_tree_status(::dhtt_msgs::msg::Subtree & msg)
  : msg_(msg)
  {}
  Init_Subtree_max_tree_depth tree_status(::dhtt_msgs::msg::Subtree::_tree_status_type arg)
  {
    msg_.tree_status = std::move(arg);
    return Init_Subtree_max_tree_depth(msg_);
  }

private:
  ::dhtt_msgs::msg::Subtree msg_;
};

class Init_Subtree_tree_nodes
{
public:
  Init_Subtree_tree_nodes()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Subtree_tree_status tree_nodes(::dhtt_msgs::msg::Subtree::_tree_nodes_type arg)
  {
    msg_.tree_nodes = std::move(arg);
    return Init_Subtree_tree_status(msg_);
  }

private:
  ::dhtt_msgs::msg::Subtree msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dhtt_msgs::msg::Subtree>()
{
  return dhtt_msgs::msg::builder::Init_Subtree_tree_nodes();
}

}  // namespace dhtt_msgs

#endif  // DHTT_MSGS__MSG__DETAIL__SUBTREE__BUILDER_HPP_
