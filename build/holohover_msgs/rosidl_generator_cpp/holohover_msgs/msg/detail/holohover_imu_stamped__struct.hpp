// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from holohover_msgs:msg/HolohoverIMUStamped.idl
// generated code does not contain a copyright notice

#ifndef HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_IMU_STAMPED__STRUCT_HPP_
#define HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_IMU_STAMPED__STRUCT_HPP_

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
// Member 'atti'
#include "holohover_msgs/msg/detail/attitude__struct.hpp"
// Member 'acc'
// Member 'gyro'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__holohover_msgs__msg__HolohoverIMUStamped __attribute__((deprecated))
#else
# define DEPRECATED__holohover_msgs__msg__HolohoverIMUStamped __declspec(deprecated)
#endif

namespace holohover_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct HolohoverIMUStamped_
{
  using Type = HolohoverIMUStamped_<ContainerAllocator>;

  explicit HolohoverIMUStamped_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    atti(_init),
    acc(_init),
    gyro(_init)
  {
    (void)_init;
  }

  explicit HolohoverIMUStamped_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    atti(_alloc, _init),
    acc(_alloc, _init),
    gyro(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _atti_type =
    holohover_msgs::msg::Attitude_<ContainerAllocator>;
  _atti_type atti;
  using _acc_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _acc_type acc;
  using _gyro_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _gyro_type gyro;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__atti(
    const holohover_msgs::msg::Attitude_<ContainerAllocator> & _arg)
  {
    this->atti = _arg;
    return *this;
  }
  Type & set__acc(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->acc = _arg;
    return *this;
  }
  Type & set__gyro(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->gyro = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    holohover_msgs::msg::HolohoverIMUStamped_<ContainerAllocator> *;
  using ConstRawPtr =
    const holohover_msgs::msg::HolohoverIMUStamped_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<holohover_msgs::msg::HolohoverIMUStamped_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<holohover_msgs::msg::HolohoverIMUStamped_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      holohover_msgs::msg::HolohoverIMUStamped_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<holohover_msgs::msg::HolohoverIMUStamped_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      holohover_msgs::msg::HolohoverIMUStamped_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<holohover_msgs::msg::HolohoverIMUStamped_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<holohover_msgs::msg::HolohoverIMUStamped_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<holohover_msgs::msg::HolohoverIMUStamped_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__holohover_msgs__msg__HolohoverIMUStamped
    std::shared_ptr<holohover_msgs::msg::HolohoverIMUStamped_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__holohover_msgs__msg__HolohoverIMUStamped
    std::shared_ptr<holohover_msgs::msg::HolohoverIMUStamped_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HolohoverIMUStamped_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->atti != other.atti) {
      return false;
    }
    if (this->acc != other.acc) {
      return false;
    }
    if (this->gyro != other.gyro) {
      return false;
    }
    return true;
  }
  bool operator!=(const HolohoverIMUStamped_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HolohoverIMUStamped_

// alias to use template instance with default allocator
using HolohoverIMUStamped =
  holohover_msgs::msg::HolohoverIMUStamped_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace holohover_msgs

#endif  // HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_IMU_STAMPED__STRUCT_HPP_
