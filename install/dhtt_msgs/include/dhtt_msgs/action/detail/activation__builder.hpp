// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dhtt_msgs:action/Activation.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__ACTION__DETAIL__ACTIVATION__BUILDER_HPP_
#define DHTT_MSGS__ACTION__DETAIL__ACTIVATION__BUILDER_HPP_

#include "dhtt_msgs/action/detail/activation__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace dhtt_msgs
{

namespace action
{

namespace builder
{

class Init_Activation_Goal_success
{
public:
  explicit Init_Activation_Goal_success(::dhtt_msgs::action::Activation_Goal & msg)
  : msg_(msg)
  {}
  ::dhtt_msgs::action::Activation_Goal success(::dhtt_msgs::action::Activation_Goal::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dhtt_msgs::action::Activation_Goal msg_;
};

class Init_Activation_Goal_granted_resources
{
public:
  explicit Init_Activation_Goal_granted_resources(::dhtt_msgs::action::Activation_Goal & msg)
  : msg_(msg)
  {}
  Init_Activation_Goal_success granted_resources(::dhtt_msgs::action::Activation_Goal::_granted_resources_type arg)
  {
    msg_.granted_resources = std::move(arg);
    return Init_Activation_Goal_success(msg_);
  }

private:
  ::dhtt_msgs::action::Activation_Goal msg_;
};

class Init_Activation_Goal_passed_resources
{
public:
  Init_Activation_Goal_passed_resources()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Activation_Goal_granted_resources passed_resources(::dhtt_msgs::action::Activation_Goal::_passed_resources_type arg)
  {
    msg_.passed_resources = std::move(arg);
    return Init_Activation_Goal_granted_resources(msg_);
  }

private:
  ::dhtt_msgs::action::Activation_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::dhtt_msgs::action::Activation_Goal>()
{
  return dhtt_msgs::action::builder::Init_Activation_Goal_passed_resources();
}

}  // namespace dhtt_msgs


namespace dhtt_msgs
{

namespace action
{

namespace builder
{

class Init_Activation_Result_passed_resources
{
public:
  explicit Init_Activation_Result_passed_resources(::dhtt_msgs::action::Activation_Result & msg)
  : msg_(msg)
  {}
  ::dhtt_msgs::action::Activation_Result passed_resources(::dhtt_msgs::action::Activation_Result::_passed_resources_type arg)
  {
    msg_.passed_resources = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dhtt_msgs::action::Activation_Result msg_;
};

class Init_Activation_Result_released_resources
{
public:
  explicit Init_Activation_Result_released_resources(::dhtt_msgs::action::Activation_Result & msg)
  : msg_(msg)
  {}
  Init_Activation_Result_passed_resources released_resources(::dhtt_msgs::action::Activation_Result::_released_resources_type arg)
  {
    msg_.released_resources = std::move(arg);
    return Init_Activation_Result_passed_resources(msg_);
  }

private:
  ::dhtt_msgs::action::Activation_Result msg_;
};

class Init_Activation_Result_activation_potential
{
public:
  explicit Init_Activation_Result_activation_potential(::dhtt_msgs::action::Activation_Result & msg)
  : msg_(msg)
  {}
  Init_Activation_Result_released_resources activation_potential(::dhtt_msgs::action::Activation_Result::_activation_potential_type arg)
  {
    msg_.activation_potential = std::move(arg);
    return Init_Activation_Result_released_resources(msg_);
  }

private:
  ::dhtt_msgs::action::Activation_Result msg_;
};

class Init_Activation_Result_possible
{
public:
  explicit Init_Activation_Result_possible(::dhtt_msgs::action::Activation_Result & msg)
  : msg_(msg)
  {}
  Init_Activation_Result_activation_potential possible(::dhtt_msgs::action::Activation_Result::_possible_type arg)
  {
    msg_.possible = std::move(arg);
    return Init_Activation_Result_activation_potential(msg_);
  }

private:
  ::dhtt_msgs::action::Activation_Result msg_;
};

class Init_Activation_Result_done
{
public:
  explicit Init_Activation_Result_done(::dhtt_msgs::action::Activation_Result & msg)
  : msg_(msg)
  {}
  Init_Activation_Result_possible done(::dhtt_msgs::action::Activation_Result::_done_type arg)
  {
    msg_.done = std::move(arg);
    return Init_Activation_Result_possible(msg_);
  }

private:
  ::dhtt_msgs::action::Activation_Result msg_;
};

class Init_Activation_Result_owned_resources
{
public:
  explicit Init_Activation_Result_owned_resources(::dhtt_msgs::action::Activation_Result & msg)
  : msg_(msg)
  {}
  Init_Activation_Result_done owned_resources(::dhtt_msgs::action::Activation_Result::_owned_resources_type arg)
  {
    msg_.owned_resources = std::move(arg);
    return Init_Activation_Result_done(msg_);
  }

private:
  ::dhtt_msgs::action::Activation_Result msg_;
};

class Init_Activation_Result_requested_resources
{
public:
  explicit Init_Activation_Result_requested_resources(::dhtt_msgs::action::Activation_Result & msg)
  : msg_(msg)
  {}
  Init_Activation_Result_owned_resources requested_resources(::dhtt_msgs::action::Activation_Result::_requested_resources_type arg)
  {
    msg_.requested_resources = std::move(arg);
    return Init_Activation_Result_owned_resources(msg_);
  }

private:
  ::dhtt_msgs::action::Activation_Result msg_;
};

class Init_Activation_Result_local_best_node
{
public:
  Init_Activation_Result_local_best_node()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Activation_Result_requested_resources local_best_node(::dhtt_msgs::action::Activation_Result::_local_best_node_type arg)
  {
    msg_.local_best_node = std::move(arg);
    return Init_Activation_Result_requested_resources(msg_);
  }

private:
  ::dhtt_msgs::action::Activation_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::dhtt_msgs::action::Activation_Result>()
{
  return dhtt_msgs::action::builder::Init_Activation_Result_local_best_node();
}

}  // namespace dhtt_msgs


namespace dhtt_msgs
{

namespace action
{


}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::dhtt_msgs::action::Activation_Feedback>()
{
  return ::dhtt_msgs::action::Activation_Feedback(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace dhtt_msgs


namespace dhtt_msgs
{

namespace action
{

namespace builder
{

class Init_Activation_SendGoal_Request_goal
{
public:
  explicit Init_Activation_SendGoal_Request_goal(::dhtt_msgs::action::Activation_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::dhtt_msgs::action::Activation_SendGoal_Request goal(::dhtt_msgs::action::Activation_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dhtt_msgs::action::Activation_SendGoal_Request msg_;
};

class Init_Activation_SendGoal_Request_goal_id
{
public:
  Init_Activation_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Activation_SendGoal_Request_goal goal_id(::dhtt_msgs::action::Activation_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Activation_SendGoal_Request_goal(msg_);
  }

private:
  ::dhtt_msgs::action::Activation_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::dhtt_msgs::action::Activation_SendGoal_Request>()
{
  return dhtt_msgs::action::builder::Init_Activation_SendGoal_Request_goal_id();
}

}  // namespace dhtt_msgs


namespace dhtt_msgs
{

namespace action
{

namespace builder
{

class Init_Activation_SendGoal_Response_stamp
{
public:
  explicit Init_Activation_SendGoal_Response_stamp(::dhtt_msgs::action::Activation_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::dhtt_msgs::action::Activation_SendGoal_Response stamp(::dhtt_msgs::action::Activation_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dhtt_msgs::action::Activation_SendGoal_Response msg_;
};

class Init_Activation_SendGoal_Response_accepted
{
public:
  Init_Activation_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Activation_SendGoal_Response_stamp accepted(::dhtt_msgs::action::Activation_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_Activation_SendGoal_Response_stamp(msg_);
  }

private:
  ::dhtt_msgs::action::Activation_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::dhtt_msgs::action::Activation_SendGoal_Response>()
{
  return dhtt_msgs::action::builder::Init_Activation_SendGoal_Response_accepted();
}

}  // namespace dhtt_msgs


namespace dhtt_msgs
{

namespace action
{

namespace builder
{

class Init_Activation_GetResult_Request_goal_id
{
public:
  Init_Activation_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dhtt_msgs::action::Activation_GetResult_Request goal_id(::dhtt_msgs::action::Activation_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dhtt_msgs::action::Activation_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::dhtt_msgs::action::Activation_GetResult_Request>()
{
  return dhtt_msgs::action::builder::Init_Activation_GetResult_Request_goal_id();
}

}  // namespace dhtt_msgs


namespace dhtt_msgs
{

namespace action
{

namespace builder
{

class Init_Activation_GetResult_Response_result
{
public:
  explicit Init_Activation_GetResult_Response_result(::dhtt_msgs::action::Activation_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::dhtt_msgs::action::Activation_GetResult_Response result(::dhtt_msgs::action::Activation_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dhtt_msgs::action::Activation_GetResult_Response msg_;
};

class Init_Activation_GetResult_Response_status
{
public:
  Init_Activation_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Activation_GetResult_Response_result status(::dhtt_msgs::action::Activation_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Activation_GetResult_Response_result(msg_);
  }

private:
  ::dhtt_msgs::action::Activation_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::dhtt_msgs::action::Activation_GetResult_Response>()
{
  return dhtt_msgs::action::builder::Init_Activation_GetResult_Response_status();
}

}  // namespace dhtt_msgs


namespace dhtt_msgs
{

namespace action
{

namespace builder
{

class Init_Activation_FeedbackMessage_feedback
{
public:
  explicit Init_Activation_FeedbackMessage_feedback(::dhtt_msgs::action::Activation_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::dhtt_msgs::action::Activation_FeedbackMessage feedback(::dhtt_msgs::action::Activation_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dhtt_msgs::action::Activation_FeedbackMessage msg_;
};

class Init_Activation_FeedbackMessage_goal_id
{
public:
  Init_Activation_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Activation_FeedbackMessage_feedback goal_id(::dhtt_msgs::action::Activation_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Activation_FeedbackMessage_feedback(msg_);
  }

private:
  ::dhtt_msgs::action::Activation_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::dhtt_msgs::action::Activation_FeedbackMessage>()
{
  return dhtt_msgs::action::builder::Init_Activation_FeedbackMessage_goal_id();
}

}  // namespace dhtt_msgs

#endif  // DHTT_MSGS__ACTION__DETAIL__ACTIVATION__BUILDER_HPP_
