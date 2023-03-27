// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rokubimini_msgs:msg/Reading.idl
// generated code does not contain a copyright notice

#ifndef ROKUBIMINI_MSGS__MSG__DETAIL__READING__STRUCT_HPP_
#define ROKUBIMINI_MSGS__MSG__DETAIL__READING__STRUCT_HPP_

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
// Member 'imu'
// Member 'external_imu'
#include "sensor_msgs/msg/detail/imu__struct.hpp"
// Member 'wrench'
#include "geometry_msgs/msg/detail/wrench_stamped__struct.hpp"
// Member 'temperature'
#include "sensor_msgs/msg/detail/temperature__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rokubimini_msgs__msg__Reading __attribute__((deprecated))
#else
# define DEPRECATED__rokubimini_msgs__msg__Reading __declspec(deprecated)
#endif

namespace rokubimini_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Reading_
{
  using Type = Reading_<ContainerAllocator>;

  explicit Reading_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    imu(_init),
    wrench(_init),
    external_imu(_init),
    temperature(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->statusword = 0ul;
      this->is_force_torque_saturated = false;
    }
  }

  explicit Reading_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    imu(_alloc, _init),
    wrench(_alloc, _init),
    external_imu(_alloc, _init),
    temperature(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->statusword = 0ul;
      this->is_force_torque_saturated = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _statusword_type =
    uint32_t;
  _statusword_type statusword;
  using _imu_type =
    sensor_msgs::msg::Imu_<ContainerAllocator>;
  _imu_type imu;
  using _wrench_type =
    geometry_msgs::msg::WrenchStamped_<ContainerAllocator>;
  _wrench_type wrench;
  using _external_imu_type =
    sensor_msgs::msg::Imu_<ContainerAllocator>;
  _external_imu_type external_imu;
  using _is_force_torque_saturated_type =
    bool;
  _is_force_torque_saturated_type is_force_torque_saturated;
  using _temperature_type =
    sensor_msgs::msg::Temperature_<ContainerAllocator>;
  _temperature_type temperature;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__statusword(
    const uint32_t & _arg)
  {
    this->statusword = _arg;
    return *this;
  }
  Type & set__imu(
    const sensor_msgs::msg::Imu_<ContainerAllocator> & _arg)
  {
    this->imu = _arg;
    return *this;
  }
  Type & set__wrench(
    const geometry_msgs::msg::WrenchStamped_<ContainerAllocator> & _arg)
  {
    this->wrench = _arg;
    return *this;
  }
  Type & set__external_imu(
    const sensor_msgs::msg::Imu_<ContainerAllocator> & _arg)
  {
    this->external_imu = _arg;
    return *this;
  }
  Type & set__is_force_torque_saturated(
    const bool & _arg)
  {
    this->is_force_torque_saturated = _arg;
    return *this;
  }
  Type & set__temperature(
    const sensor_msgs::msg::Temperature_<ContainerAllocator> & _arg)
  {
    this->temperature = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rokubimini_msgs::msg::Reading_<ContainerAllocator> *;
  using ConstRawPtr =
    const rokubimini_msgs::msg::Reading_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rokubimini_msgs::msg::Reading_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rokubimini_msgs::msg::Reading_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rokubimini_msgs::msg::Reading_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rokubimini_msgs::msg::Reading_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rokubimini_msgs::msg::Reading_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rokubimini_msgs::msg::Reading_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rokubimini_msgs::msg::Reading_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rokubimini_msgs::msg::Reading_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rokubimini_msgs__msg__Reading
    std::shared_ptr<rokubimini_msgs::msg::Reading_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rokubimini_msgs__msg__Reading
    std::shared_ptr<rokubimini_msgs::msg::Reading_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Reading_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->statusword != other.statusword) {
      return false;
    }
    if (this->imu != other.imu) {
      return false;
    }
    if (this->wrench != other.wrench) {
      return false;
    }
    if (this->external_imu != other.external_imu) {
      return false;
    }
    if (this->is_force_torque_saturated != other.is_force_torque_saturated) {
      return false;
    }
    if (this->temperature != other.temperature) {
      return false;
    }
    return true;
  }
  bool operator!=(const Reading_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Reading_

// alias to use template instance with default allocator
using Reading =
  rokubimini_msgs::msg::Reading_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rokubimini_msgs

#endif  // ROKUBIMINI_MSGS__MSG__DETAIL__READING__STRUCT_HPP_
