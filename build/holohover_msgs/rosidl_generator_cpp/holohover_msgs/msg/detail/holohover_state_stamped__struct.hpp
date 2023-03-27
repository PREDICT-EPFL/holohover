// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from holohover_msgs:msg/HolohoverStateStamped.idl
// generated code does not contain a copyright notice

#ifndef HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_STATE_STAMPED__STRUCT_HPP_
#define HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_STATE_STAMPED__STRUCT_HPP_

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
# define DEPRECATED__holohover_msgs__msg__HolohoverStateStamped __attribute__((deprecated))
#else
# define DEPRECATED__holohover_msgs__msg__HolohoverStateStamped __declspec(deprecated)
#endif

namespace holohover_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct HolohoverStateStamped_
{
  using Type = HolohoverStateStamped_<ContainerAllocator>;

  explicit HolohoverStateStamped_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
      this->v_x = 0.0;
      this->v_y = 0.0;
      this->yaw = 0.0;
      this->w_z = 0.0;
    }
  }

  explicit HolohoverStateStamped_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
      this->v_x = 0.0;
      this->v_y = 0.0;
      this->yaw = 0.0;
      this->w_z = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _x_type =
    double;
  _x_type x;
  using _y_type =
    double;
  _y_type y;
  using _v_x_type =
    double;
  _v_x_type v_x;
  using _v_y_type =
    double;
  _v_y_type v_y;
  using _yaw_type =
    double;
  _yaw_type yaw;
  using _w_z_type =
    double;
  _w_z_type w_z;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__x(
    const double & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const double & _arg)
  {
    this->y = _arg;
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
  Type & set__yaw(
    const double & _arg)
  {
    this->yaw = _arg;
    return *this;
  }
  Type & set__w_z(
    const double & _arg)
  {
    this->w_z = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    holohover_msgs::msg::HolohoverStateStamped_<ContainerAllocator> *;
  using ConstRawPtr =
    const holohover_msgs::msg::HolohoverStateStamped_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<holohover_msgs::msg::HolohoverStateStamped_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<holohover_msgs::msg::HolohoverStateStamped_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      holohover_msgs::msg::HolohoverStateStamped_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<holohover_msgs::msg::HolohoverStateStamped_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      holohover_msgs::msg::HolohoverStateStamped_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<holohover_msgs::msg::HolohoverStateStamped_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<holohover_msgs::msg::HolohoverStateStamped_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<holohover_msgs::msg::HolohoverStateStamped_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__holohover_msgs__msg__HolohoverStateStamped
    std::shared_ptr<holohover_msgs::msg::HolohoverStateStamped_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__holohover_msgs__msg__HolohoverStateStamped
    std::shared_ptr<holohover_msgs::msg::HolohoverStateStamped_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HolohoverStateStamped_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->v_x != other.v_x) {
      return false;
    }
    if (this->v_y != other.v_y) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    if (this->w_z != other.w_z) {
      return false;
    }
    return true;
  }
  bool operator!=(const HolohoverStateStamped_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HolohoverStateStamped_

// alias to use template instance with default allocator
using HolohoverStateStamped =
  holohover_msgs::msg::HolohoverStateStamped_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace holohover_msgs

#endif  // HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_STATE_STAMPED__STRUCT_HPP_
