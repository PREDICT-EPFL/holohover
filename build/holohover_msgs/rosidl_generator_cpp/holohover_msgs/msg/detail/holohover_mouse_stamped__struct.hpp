// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from holohover_msgs:msg/HolohoverMouseStamped.idl
// generated code does not contain a copyright notice

#ifndef HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_MOUSE_STAMPED__STRUCT_HPP_
#define HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_MOUSE_STAMPED__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__holohover_msgs__msg__HolohoverMouseStamped __attribute__((deprecated))
#else
# define DEPRECATED__holohover_msgs__msg__HolohoverMouseStamped __declspec(deprecated)
#endif

namespace holohover_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct HolohoverMouseStamped_
{
  using Type = HolohoverMouseStamped_<ContainerAllocator>;

  explicit HolohoverMouseStamped_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->v_x = 0.0;
      this->v_y = 0.0;
    }
  }

  explicit HolohoverMouseStamped_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->v_x = 0.0;
      this->v_y = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _v_x_type =
    double;
  _v_x_type v_x;
  using _v_y_type =
    double;
  _v_y_type v_y;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__v_x(
    const double & _arg)
  {
    this->v_x = _arg;
    return *this;
  }
  Type & set__v_y(
    const double & _arg)
  {
    this->v_y = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    holohover_msgs::msg::HolohoverMouseStamped_<ContainerAllocator> *;
  using ConstRawPtr =
    const holohover_msgs::msg::HolohoverMouseStamped_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<holohover_msgs::msg::HolohoverMouseStamped_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<holohover_msgs::msg::HolohoverMouseStamped_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      holohover_msgs::msg::HolohoverMouseStamped_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<holohover_msgs::msg::HolohoverMouseStamped_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      holohover_msgs::msg::HolohoverMouseStamped_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<holohover_msgs::msg::HolohoverMouseStamped_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<holohover_msgs::msg::HolohoverMouseStamped_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<holohover_msgs::msg::HolohoverMouseStamped_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__holohover_msgs__msg__HolohoverMouseStamped
    std::shared_ptr<holohover_msgs::msg::HolohoverMouseStamped_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__holohover_msgs__msg__HolohoverMouseStamped
    std::shared_ptr<holohover_msgs::msg::HolohoverMouseStamped_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HolohoverMouseStamped_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->v_x != other.v_x) {
      return false;
    }
    if (this->v_y != other.v_y) {
      return false;
    }
    return true;
  }
  bool operator!=(const HolohoverMouseStamped_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HolohoverMouseStamped_

// alias to use template instance with default allocator
using HolohoverMouseStamped =
  holohover_msgs::msg::HolohoverMouseStamped_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace holohover_msgs

#endif  // HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_MOUSE_STAMPED__STRUCT_HPP_
