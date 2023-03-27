// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rokubimini_msgs:srv/ResetWrench.idl
// generated code does not contain a copyright notice

#ifndef ROKUBIMINI_MSGS__SRV__DETAIL__RESET_WRENCH__STRUCT_HPP_
#define ROKUBIMINI_MSGS__SRV__DETAIL__RESET_WRENCH__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'desired_wrench'
#include "geometry_msgs/msg/detail/wrench__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rokubimini_msgs__srv__ResetWrench_Request __attribute__((deprecated))
#else
# define DEPRECATED__rokubimini_msgs__srv__ResetWrench_Request __declspec(deprecated)
#endif

namespace rokubimini_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ResetWrench_Request_
{
  using Type = ResetWrench_Request_<ContainerAllocator>;

  explicit ResetWrench_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : desired_wrench(_init)
  {
    (void)_init;
  }

  explicit ResetWrench_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : desired_wrench(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _desired_wrench_type =
    geometry_msgs::msg::Wrench_<ContainerAllocator>;
  _desired_wrench_type desired_wrench;

  // setters for named parameter idiom
  Type & set__desired_wrench(
    const geometry_msgs::msg::Wrench_<ContainerAllocator> & _arg)
  {
    this->desired_wrench = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rokubimini_msgs::srv::ResetWrench_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const rokubimini_msgs::srv::ResetWrench_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rokubimini_msgs::srv::ResetWrench_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rokubimini_msgs::srv::ResetWrench_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rokubimini_msgs::srv::ResetWrench_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rokubimini_msgs::srv::ResetWrench_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rokubimini_msgs::srv::ResetWrench_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rokubimini_msgs::srv::ResetWrench_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rokubimini_msgs::srv::ResetWrench_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rokubimini_msgs::srv::ResetWrench_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rokubimini_msgs__srv__ResetWrench_Request
    std::shared_ptr<rokubimini_msgs::srv::ResetWrench_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rokubimini_msgs__srv__ResetWrench_Request
    std::shared_ptr<rokubimini_msgs::srv::ResetWrench_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ResetWrench_Request_ & other) const
  {
    if (this->desired_wrench != other.desired_wrench) {
      return false;
    }
    return true;
  }
  bool operator!=(const ResetWrench_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ResetWrench_Request_

// alias to use template instance with default allocator
using ResetWrench_Request =
  rokubimini_msgs::srv::ResetWrench_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rokubimini_msgs


#ifndef _WIN32
# define DEPRECATED__rokubimini_msgs__srv__ResetWrench_Response __attribute__((deprecated))
#else
# define DEPRECATED__rokubimini_msgs__srv__ResetWrench_Response __declspec(deprecated)
#endif

namespace rokubimini_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ResetWrench_Response_
{
  using Type = ResetWrench_Response_<ContainerAllocator>;

  explicit ResetWrench_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit ResetWrench_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rokubimini_msgs::srv::ResetWrench_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const rokubimini_msgs::srv::ResetWrench_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rokubimini_msgs::srv::ResetWrench_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rokubimini_msgs::srv::ResetWrench_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rokubimini_msgs::srv::ResetWrench_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rokubimini_msgs::srv::ResetWrench_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rokubimini_msgs::srv::ResetWrench_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rokubimini_msgs::srv::ResetWrench_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rokubimini_msgs::srv::ResetWrench_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rokubimini_msgs::srv::ResetWrench_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rokubimini_msgs__srv__ResetWrench_Response
    std::shared_ptr<rokubimini_msgs::srv::ResetWrench_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rokubimini_msgs__srv__ResetWrench_Response
    std::shared_ptr<rokubimini_msgs::srv::ResetWrench_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ResetWrench_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const ResetWrench_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ResetWrench_Response_

// alias to use template instance with default allocator
using ResetWrench_Response =
  rokubimini_msgs::srv::ResetWrench_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rokubimini_msgs

namespace rokubimini_msgs
{

namespace srv
{

struct ResetWrench
{
  using Request = rokubimini_msgs::srv::ResetWrench_Request;
  using Response = rokubimini_msgs::srv::ResetWrench_Response;
};

}  // namespace srv

}  // namespace rokubimini_msgs

#endif  // ROKUBIMINI_MSGS__SRV__DETAIL__RESET_WRENCH__STRUCT_HPP_
