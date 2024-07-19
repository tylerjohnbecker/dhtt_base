// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dhtt_msgs:action/Activation.idl
// generated code does not contain a copyright notice

#ifndef DHTT_MSGS__ACTION__DETAIL__ACTIVATION__STRUCT_HPP_
#define DHTT_MSGS__ACTION__DETAIL__ACTIVATION__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'passed_resources'
// Member 'granted_resources'
#include "dhtt_msgs/msg/detail/resource__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__dhtt_msgs__action__Activation_Goal __attribute__((deprecated))
#else
# define DEPRECATED__dhtt_msgs__action__Activation_Goal __declspec(deprecated)
#endif

namespace dhtt_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Activation_Goal_
{
  using Type = Activation_Goal_<ContainerAllocator>;

  explicit Activation_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit Activation_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _passed_resources_type =
    std::vector<dhtt_msgs::msg::Resource_<ContainerAllocator>, typename ContainerAllocator::template rebind<dhtt_msgs::msg::Resource_<ContainerAllocator>>::other>;
  _passed_resources_type passed_resources;
  using _granted_resources_type =
    std::vector<dhtt_msgs::msg::Resource_<ContainerAllocator>, typename ContainerAllocator::template rebind<dhtt_msgs::msg::Resource_<ContainerAllocator>>::other>;
  _granted_resources_type granted_resources;
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__passed_resources(
    const std::vector<dhtt_msgs::msg::Resource_<ContainerAllocator>, typename ContainerAllocator::template rebind<dhtt_msgs::msg::Resource_<ContainerAllocator>>::other> & _arg)
  {
    this->passed_resources = _arg;
    return *this;
  }
  Type & set__granted_resources(
    const std::vector<dhtt_msgs::msg::Resource_<ContainerAllocator>, typename ContainerAllocator::template rebind<dhtt_msgs::msg::Resource_<ContainerAllocator>>::other> & _arg)
  {
    this->granted_resources = _arg;
    return *this;
  }
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dhtt_msgs::action::Activation_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const dhtt_msgs::action::Activation_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dhtt_msgs::action::Activation_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dhtt_msgs::action::Activation_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::action::Activation_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::action::Activation_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::action::Activation_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::action::Activation_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dhtt_msgs::action::Activation_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dhtt_msgs::action::Activation_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dhtt_msgs__action__Activation_Goal
    std::shared_ptr<dhtt_msgs::action::Activation_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dhtt_msgs__action__Activation_Goal
    std::shared_ptr<dhtt_msgs::action::Activation_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Activation_Goal_ & other) const
  {
    if (this->passed_resources != other.passed_resources) {
      return false;
    }
    if (this->granted_resources != other.granted_resources) {
      return false;
    }
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const Activation_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Activation_Goal_

// alias to use template instance with default allocator
using Activation_Goal =
  dhtt_msgs::action::Activation_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace dhtt_msgs


// Include directives for member types
// Member 'requested_resources'
// Member 'owned_resources'
// Member 'released_resources'
// Member 'passed_resources'
// already included above
// #include "dhtt_msgs/msg/detail/resource__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__dhtt_msgs__action__Activation_Result __attribute__((deprecated))
#else
# define DEPRECATED__dhtt_msgs__action__Activation_Result __declspec(deprecated)
#endif

namespace dhtt_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Activation_Result_
{
  using Type = Activation_Result_<ContainerAllocator>;

  explicit Activation_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->local_best_node = "";
      this->done = false;
      this->possible = false;
      this->activation_potential = 0.0;
    }
  }

  explicit Activation_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : local_best_node(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->local_best_node = "";
      this->done = false;
      this->possible = false;
      this->activation_potential = 0.0;
    }
  }

  // field types and members
  using _local_best_node_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _local_best_node_type local_best_node;
  using _requested_resources_type =
    std::vector<dhtt_msgs::msg::Resource_<ContainerAllocator>, typename ContainerAllocator::template rebind<dhtt_msgs::msg::Resource_<ContainerAllocator>>::other>;
  _requested_resources_type requested_resources;
  using _owned_resources_type =
    std::vector<dhtt_msgs::msg::Resource_<ContainerAllocator>, typename ContainerAllocator::template rebind<dhtt_msgs::msg::Resource_<ContainerAllocator>>::other>;
  _owned_resources_type owned_resources;
  using _done_type =
    bool;
  _done_type done;
  using _possible_type =
    bool;
  _possible_type possible;
  using _activation_potential_type =
    double;
  _activation_potential_type activation_potential;
  using _released_resources_type =
    std::vector<dhtt_msgs::msg::Resource_<ContainerAllocator>, typename ContainerAllocator::template rebind<dhtt_msgs::msg::Resource_<ContainerAllocator>>::other>;
  _released_resources_type released_resources;
  using _passed_resources_type =
    std::vector<dhtt_msgs::msg::Resource_<ContainerAllocator>, typename ContainerAllocator::template rebind<dhtt_msgs::msg::Resource_<ContainerAllocator>>::other>;
  _passed_resources_type passed_resources;

  // setters for named parameter idiom
  Type & set__local_best_node(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->local_best_node = _arg;
    return *this;
  }
  Type & set__requested_resources(
    const std::vector<dhtt_msgs::msg::Resource_<ContainerAllocator>, typename ContainerAllocator::template rebind<dhtt_msgs::msg::Resource_<ContainerAllocator>>::other> & _arg)
  {
    this->requested_resources = _arg;
    return *this;
  }
  Type & set__owned_resources(
    const std::vector<dhtt_msgs::msg::Resource_<ContainerAllocator>, typename ContainerAllocator::template rebind<dhtt_msgs::msg::Resource_<ContainerAllocator>>::other> & _arg)
  {
    this->owned_resources = _arg;
    return *this;
  }
  Type & set__done(
    const bool & _arg)
  {
    this->done = _arg;
    return *this;
  }
  Type & set__possible(
    const bool & _arg)
  {
    this->possible = _arg;
    return *this;
  }
  Type & set__activation_potential(
    const double & _arg)
  {
    this->activation_potential = _arg;
    return *this;
  }
  Type & set__released_resources(
    const std::vector<dhtt_msgs::msg::Resource_<ContainerAllocator>, typename ContainerAllocator::template rebind<dhtt_msgs::msg::Resource_<ContainerAllocator>>::other> & _arg)
  {
    this->released_resources = _arg;
    return *this;
  }
  Type & set__passed_resources(
    const std::vector<dhtt_msgs::msg::Resource_<ContainerAllocator>, typename ContainerAllocator::template rebind<dhtt_msgs::msg::Resource_<ContainerAllocator>>::other> & _arg)
  {
    this->passed_resources = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dhtt_msgs::action::Activation_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const dhtt_msgs::action::Activation_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dhtt_msgs::action::Activation_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dhtt_msgs::action::Activation_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::action::Activation_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::action::Activation_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::action::Activation_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::action::Activation_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dhtt_msgs::action::Activation_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dhtt_msgs::action::Activation_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dhtt_msgs__action__Activation_Result
    std::shared_ptr<dhtt_msgs::action::Activation_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dhtt_msgs__action__Activation_Result
    std::shared_ptr<dhtt_msgs::action::Activation_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Activation_Result_ & other) const
  {
    if (this->local_best_node != other.local_best_node) {
      return false;
    }
    if (this->requested_resources != other.requested_resources) {
      return false;
    }
    if (this->owned_resources != other.owned_resources) {
      return false;
    }
    if (this->done != other.done) {
      return false;
    }
    if (this->possible != other.possible) {
      return false;
    }
    if (this->activation_potential != other.activation_potential) {
      return false;
    }
    if (this->released_resources != other.released_resources) {
      return false;
    }
    if (this->passed_resources != other.passed_resources) {
      return false;
    }
    return true;
  }
  bool operator!=(const Activation_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Activation_Result_

// alias to use template instance with default allocator
using Activation_Result =
  dhtt_msgs::action::Activation_Result_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace dhtt_msgs


#ifndef _WIN32
# define DEPRECATED__dhtt_msgs__action__Activation_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__dhtt_msgs__action__Activation_Feedback __declspec(deprecated)
#endif

namespace dhtt_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Activation_Feedback_
{
  using Type = Activation_Feedback_<ContainerAllocator>;

  explicit Activation_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit Activation_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    dhtt_msgs::action::Activation_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const dhtt_msgs::action::Activation_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dhtt_msgs::action::Activation_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dhtt_msgs::action::Activation_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::action::Activation_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::action::Activation_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::action::Activation_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::action::Activation_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dhtt_msgs::action::Activation_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dhtt_msgs::action::Activation_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dhtt_msgs__action__Activation_Feedback
    std::shared_ptr<dhtt_msgs::action::Activation_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dhtt_msgs__action__Activation_Feedback
    std::shared_ptr<dhtt_msgs::action::Activation_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Activation_Feedback_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const Activation_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Activation_Feedback_

// alias to use template instance with default allocator
using Activation_Feedback =
  dhtt_msgs::action::Activation_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace dhtt_msgs


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "dhtt_msgs/action/detail/activation__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__dhtt_msgs__action__Activation_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__dhtt_msgs__action__Activation_SendGoal_Request __declspec(deprecated)
#endif

namespace dhtt_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Activation_SendGoal_Request_
{
  using Type = Activation_SendGoal_Request_<ContainerAllocator>;

  explicit Activation_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit Activation_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    goal(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _goal_type =
    dhtt_msgs::action::Activation_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const dhtt_msgs::action::Activation_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dhtt_msgs::action::Activation_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const dhtt_msgs::action::Activation_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dhtt_msgs::action::Activation_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dhtt_msgs::action::Activation_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::action::Activation_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::action::Activation_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::action::Activation_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::action::Activation_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dhtt_msgs::action::Activation_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dhtt_msgs::action::Activation_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dhtt_msgs__action__Activation_SendGoal_Request
    std::shared_ptr<dhtt_msgs::action::Activation_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dhtt_msgs__action__Activation_SendGoal_Request
    std::shared_ptr<dhtt_msgs::action::Activation_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Activation_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const Activation_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Activation_SendGoal_Request_

// alias to use template instance with default allocator
using Activation_SendGoal_Request =
  dhtt_msgs::action::Activation_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace dhtt_msgs


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__dhtt_msgs__action__Activation_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__dhtt_msgs__action__Activation_SendGoal_Response __declspec(deprecated)
#endif

namespace dhtt_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Activation_SendGoal_Response_
{
  using Type = Activation_SendGoal_Response_<ContainerAllocator>;

  explicit Activation_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit Activation_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dhtt_msgs::action::Activation_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const dhtt_msgs::action::Activation_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dhtt_msgs::action::Activation_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dhtt_msgs::action::Activation_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::action::Activation_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::action::Activation_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::action::Activation_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::action::Activation_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dhtt_msgs::action::Activation_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dhtt_msgs::action::Activation_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dhtt_msgs__action__Activation_SendGoal_Response
    std::shared_ptr<dhtt_msgs::action::Activation_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dhtt_msgs__action__Activation_SendGoal_Response
    std::shared_ptr<dhtt_msgs::action::Activation_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Activation_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const Activation_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Activation_SendGoal_Response_

// alias to use template instance with default allocator
using Activation_SendGoal_Response =
  dhtt_msgs::action::Activation_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace dhtt_msgs

namespace dhtt_msgs
{

namespace action
{

struct Activation_SendGoal
{
  using Request = dhtt_msgs::action::Activation_SendGoal_Request;
  using Response = dhtt_msgs::action::Activation_SendGoal_Response;
};

}  // namespace action

}  // namespace dhtt_msgs


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__dhtt_msgs__action__Activation_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__dhtt_msgs__action__Activation_GetResult_Request __declspec(deprecated)
#endif

namespace dhtt_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Activation_GetResult_Request_
{
  using Type = Activation_GetResult_Request_<ContainerAllocator>;

  explicit Activation_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit Activation_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dhtt_msgs::action::Activation_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const dhtt_msgs::action::Activation_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dhtt_msgs::action::Activation_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dhtt_msgs::action::Activation_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::action::Activation_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::action::Activation_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::action::Activation_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::action::Activation_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dhtt_msgs::action::Activation_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dhtt_msgs::action::Activation_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dhtt_msgs__action__Activation_GetResult_Request
    std::shared_ptr<dhtt_msgs::action::Activation_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dhtt_msgs__action__Activation_GetResult_Request
    std::shared_ptr<dhtt_msgs::action::Activation_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Activation_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const Activation_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Activation_GetResult_Request_

// alias to use template instance with default allocator
using Activation_GetResult_Request =
  dhtt_msgs::action::Activation_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace dhtt_msgs


// Include directives for member types
// Member 'result'
// already included above
// #include "dhtt_msgs/action/detail/activation__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__dhtt_msgs__action__Activation_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__dhtt_msgs__action__Activation_GetResult_Response __declspec(deprecated)
#endif

namespace dhtt_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Activation_GetResult_Response_
{
  using Type = Activation_GetResult_Response_<ContainerAllocator>;

  explicit Activation_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit Activation_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    int8_t;
  _status_type status;
  using _result_type =
    dhtt_msgs::action::Activation_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const dhtt_msgs::action::Activation_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dhtt_msgs::action::Activation_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const dhtt_msgs::action::Activation_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dhtt_msgs::action::Activation_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dhtt_msgs::action::Activation_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::action::Activation_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::action::Activation_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::action::Activation_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::action::Activation_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dhtt_msgs::action::Activation_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dhtt_msgs::action::Activation_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dhtt_msgs__action__Activation_GetResult_Response
    std::shared_ptr<dhtt_msgs::action::Activation_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dhtt_msgs__action__Activation_GetResult_Response
    std::shared_ptr<dhtt_msgs::action::Activation_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Activation_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const Activation_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Activation_GetResult_Response_

// alias to use template instance with default allocator
using Activation_GetResult_Response =
  dhtt_msgs::action::Activation_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace dhtt_msgs

namespace dhtt_msgs
{

namespace action
{

struct Activation_GetResult
{
  using Request = dhtt_msgs::action::Activation_GetResult_Request;
  using Response = dhtt_msgs::action::Activation_GetResult_Response;
};

}  // namespace action

}  // namespace dhtt_msgs


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "dhtt_msgs/action/detail/activation__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__dhtt_msgs__action__Activation_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__dhtt_msgs__action__Activation_FeedbackMessage __declspec(deprecated)
#endif

namespace dhtt_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Activation_FeedbackMessage_
{
  using Type = Activation_FeedbackMessage_<ContainerAllocator>;

  explicit Activation_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit Activation_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    feedback(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _feedback_type =
    dhtt_msgs::action::Activation_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const dhtt_msgs::action::Activation_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dhtt_msgs::action::Activation_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const dhtt_msgs::action::Activation_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dhtt_msgs::action::Activation_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dhtt_msgs::action::Activation_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::action::Activation_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::action::Activation_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dhtt_msgs::action::Activation_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dhtt_msgs::action::Activation_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dhtt_msgs::action::Activation_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dhtt_msgs::action::Activation_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dhtt_msgs__action__Activation_FeedbackMessage
    std::shared_ptr<dhtt_msgs::action::Activation_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dhtt_msgs__action__Activation_FeedbackMessage
    std::shared_ptr<dhtt_msgs::action::Activation_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Activation_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const Activation_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Activation_FeedbackMessage_

// alias to use template instance with default allocator
using Activation_FeedbackMessage =
  dhtt_msgs::action::Activation_FeedbackMessage_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace dhtt_msgs

#include "action_msgs/srv/cancel_goal.hpp"
#include "action_msgs/msg/goal_info.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

namespace dhtt_msgs
{

namespace action
{

struct Activation
{
  /// The goal message defined in the action definition.
  using Goal = dhtt_msgs::action::Activation_Goal;
  /// The result message defined in the action definition.
  using Result = dhtt_msgs::action::Activation_Result;
  /// The feedback message defined in the action definition.
  using Feedback = dhtt_msgs::action::Activation_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = dhtt_msgs::action::Activation_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = dhtt_msgs::action::Activation_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = dhtt_msgs::action::Activation_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct Activation Activation;

}  // namespace action

}  // namespace dhtt_msgs

#endif  // DHTT_MSGS__ACTION__DETAIL__ACTIVATION__STRUCT_HPP_
