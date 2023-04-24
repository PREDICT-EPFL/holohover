// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from holohover_msgs:msg/HolohoverIMUStamped.idl
// generated code does not contain a copyright notice

#ifndef HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_IMU_STAMPED__BUILDER_HPP_
#define HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_IMU_STAMPED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "holohover_msgs/msg/detail/holohover_imu_stamped__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace holohover_msgs
{

namespace msg
{

namespace builder
{

class Init_HolohoverIMUStamped_gyro
{
public:
  explicit Init_HolohoverIMUStamped_gyro(::holohover_msgs::msg::HolohoverIMUStamped & msg)
  : msg_(msg)
  {}
  ::holohover_msgs::msg::HolohoverIMUStamped gyro(::holohover_msgs::msg::HolohoverIMUStamped::_gyro_type arg)
  {
    msg_.gyro = std::move(arg);
    return std::move(msg_);
  }

private:
  ::holohover_msgs::msg::HolohoverIMUStamped msg_;
};

class Init_HolohoverIMUStamped_acc
{
public:
  explicit Init_HolohoverIMUStamped_acc(::holohover_msgs::msg::HolohoverIMUStamped & msg)
  : msg_(msg)
  {}
  Init_HolohoverIMUStamped_gyro acc(::holohover_msgs::msg::HolohoverIMUStamped::_acc_type arg)
  {
    msg_.acc = std::move(arg);
    return Init_HolohoverIMUStamped_gyro(msg_);
  }

private:
  ::holohover_msgs::msg::HolohoverIMUStamped msg_;
};

class Init_HolohoverIMUStamped_atti
{
public:
  explicit Init_HolohoverIMUStamped_atti(::holohover_msgs::msg::HolohoverIMUStamped & msg)
  : msg_(msg)
  {}
  Init_HolohoverIMUStamped_acc atti(::holohover_msgs::msg::HolohoverIMUStamped::_atti_type arg)
  {
    msg_.atti = std::move(arg);
    return Init_HolohoverIMUStamped_acc(msg_);
  }

private:
  ::holohover_msgs::msg::HolohoverIMUStamped msg_;
};

class Init_HolohoverIMUStamped_header
{
public:
  Init_HolohoverIMUStamped_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HolohoverIMUStamped_atti header(::holohover_msgs::msg::HolohoverIMUStamped::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_HolohoverIMUStamped_atti(msg_);
  }

private:
  ::holohover_msgs::msg::HolohoverIMUStamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::holohover_msgs::msg::HolohoverIMUStamped>()
{
  return holohover_msgs::msg::builder::Init_HolohoverIMUStamped_header();
}

}  // namespace holohover_msgs

#endif  // HOLOHOVER_MSGS__MSG__DETAIL__HOLOHOVER_IMU_STAMPED__BUILDER_HPP_
