// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from holohover_msgs:msg/HolohoverControlStamped.idl
// generated code does not contain a copyright notice

#ifndef HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_CONTROL_STAMPED__STRUCT_HPP_
#define HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_CONTROL_STAMPED__STRUCT_HPP_

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
# define DEPRECATED__holohover_msgs__msg__HolohoverControlStamped __attribute__((deprecated))
#else
# define DEPRECATED__holohover_msgs__msg__HolohoverControlStamped __declspec(deprecated)
#endif

namespace holohover_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct HolohoverControlStamped_
{
  using Type = HolohoverControlStamped_<ContainerAllocator>;

  explicit HolohoverControlStamped_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->motor_a_1 = 0.0;
      this->motor_a_2 = 0.0;
      this->motor_b_1 = 0.0;
      this->motor_b_2 = 0.0;
      this->motor_c_1 = 0.0;
      this->motor_c_2 = 0.0;
    }
  }

  explicit HolohoverControlStamped_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->motor_a_1 = 0.0;
      this->motor_a_2 = 0.0;
      this->motor_b_1 = 0.0;
      this->motor_b_2 = 0.0;
      this->motor_c_1 = 0.0;
      this->motor_c_2 = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _motor_a_1_type =
    double;
  _motor_a_1_type motor_a_1;
  using _motor_a_2_type =
    double;
  _motor_a_2_type motor_a_2;
  using _motor_b_1_type =
    double;
  _motor_b_1_type motor_b_1;
  using _motor_b_2_type =
    double;
  _motor_b_2_type motor_b_2;
  using _motor_c_1_type =
    double;
  _motor_c_1_type motor_c_1;
  using _motor_c_2_type =
    double;
  _motor_c_2_type motor_c_2;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__motor_a_1(
    const double & _arg)
  {
    this->motor_a_1 = _arg;
    return *this;
  }
  Type & set__motor_a_2(
    const double & _arg)
  {
    this->motor_a_2 = _arg;
    return *this;
  }
  Type & set__motor_b_1(
    const double & _arg)
  {
    this->motor_b_1 = _arg;
    return *this;
  }
  Type & set__motor_b_2(
    const double & _arg)
  {
    this->motor_b_2 = _arg;
    return *this;
  }
  Type & set__motor_c_1(
    const double & _arg)
  {
    this->motor_c_1 = _arg;
    return *this;
  }
  Type & set__motor_c_2(
    const double & _arg)
  {
    this->motor_c_2 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    holohover_msgs::msg::HolohoverControlStamped_<ContainerAllocator> *;
  using ConstRawPtr =
    const holohover_msgs::msg::HolohoverControlStamped_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<holohover_msgs::msg::HolohoverControlStamped_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<holohover_msgs::msg::HolohoverControlStamped_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      holohover_msgs::msg::HolohoverControlStamped_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<holohover_msgs::msg::HolohoverControlStamped_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      holohover_msgs::msg::HolohoverControlStamped_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<holohover_msgs::msg::HolohoverControlStamped_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<holohover_msgs::msg::HolohoverControlStamped_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<holohover_msgs::msg::HolohoverControlStamped_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__holohover_msgs__msg__HolohoverControlStamped
    std::shared_ptr<holohover_msgs::msg::HolohoverControlStamped_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__holohover_msgs__msg__HolohoverControlStamped
    std::shared_ptr<holohover_msgs::msg::HolohoverControlStamped_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HolohoverControlStamped_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->motor_a_1 != other.motor_a_1) {
      return false;
    }
    if (this->motor_a_2 != other.motor_a_2) {
      return false;
    }
    if (this->motor_b_1 != other.motor_b_1) {
      return false;
    }
    if (this->motor_b_2 != other.motor_b_2) {
      return false;
    }
    if (this->motor_c_1 != other.motor_c_1) {
      return false;
    }
    if (this->motor_c_2 != other.motor_c_2) {
      return false;
    }
    return true;
  }
  bool operator!=(const HolohoverControlStamped_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HolohoverControlStamped_

// alias to use template instance with default allocator
using HolohoverControlStamped =
  holohover_msgs::msg::HolohoverControlStamped_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace holohover_msgs

#endif  // HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_CONTROL_STAMPED__STRUCT_HPP_
