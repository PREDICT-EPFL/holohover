// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rokubimini_msgs:srv/FirmwareUpdateSerial.idl
// generated code does not contain a copyright notice

#ifndef ROKUBIMINI_MSGS__SRV__DETAIL__FIRMWARE_UPDATE_SERIAL__BUILDER_HPP_
#define ROKUBIMINI_MSGS__SRV__DETAIL__FIRMWARE_UPDATE_SERIAL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rokubimini_msgs/srv/detail/firmware_update_serial__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rokubimini_msgs
{

namespace srv
{

namespace builder
{

class Init_FirmwareUpdateSerial_Request_file_path
{
public:
  Init_FirmwareUpdateSerial_Request_file_path()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rokubimini_msgs::srv::FirmwareUpdateSerial_Request file_path(::rokubimini_msgs::srv::FirmwareUpdateSerial_Request::_file_path_type arg)
  {
    msg_.file_path = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rokubimini_msgs::srv::FirmwareUpdateSerial_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rokubimini_msgs::srv::FirmwareUpdateSerial_Request>()
{
  return rokubimini_msgs::srv::builder::Init_FirmwareUpdateSerial_Request_file_path();
}

}  // namespace rokubimini_msgs


namespace rokubimini_msgs
{

namespace srv
{

namespace builder
{

class Init_FirmwareUpdateSerial_Response_result
{
public:
  Init_FirmwareUpdateSerial_Response_result()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rokubimini_msgs::srv::FirmwareUpdateSerial_Response result(::rokubimini_msgs::srv::FirmwareUpdateSerial_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rokubimini_msgs::srv::FirmwareUpdateSerial_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rokubimini_msgs::srv::FirmwareUpdateSerial_Response>()
{
  return rokubimini_msgs::srv::builder::Init_FirmwareUpdateSerial_Response_result();
}

}  // namespace rokubimini_msgs

#endif  // ROKUBIMINI_MSGS__SRV__DETAIL__FIRMWARE_UPDATE_SERIAL__BUILDER_HPP_
