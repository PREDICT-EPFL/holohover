// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rokubimini_msgs:srv/FirmwareUpdateEthercat.idl
// generated code does not contain a copyright notice

#ifndef ROKUBIMINI_MSGS__SRV__DETAIL__FIRMWARE_UPDATE_ETHERCAT__BUILDER_HPP_
#define ROKUBIMINI_MSGS__SRV__DETAIL__FIRMWARE_UPDATE_ETHERCAT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rokubimini_msgs/srv/detail/firmware_update_ethercat__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rokubimini_msgs
{

namespace srv
{

namespace builder
{

class Init_FirmwareUpdateEthercat_Request_password
{
public:
  explicit Init_FirmwareUpdateEthercat_Request_password(::rokubimini_msgs::srv::FirmwareUpdateEthercat_Request & msg)
  : msg_(msg)
  {}
  ::rokubimini_msgs::srv::FirmwareUpdateEthercat_Request password(::rokubimini_msgs::srv::FirmwareUpdateEthercat_Request::_password_type arg)
  {
    msg_.password = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rokubimini_msgs::srv::FirmwareUpdateEthercat_Request msg_;
};

class Init_FirmwareUpdateEthercat_Request_file_path
{
public:
  explicit Init_FirmwareUpdateEthercat_Request_file_path(::rokubimini_msgs::srv::FirmwareUpdateEthercat_Request & msg)
  : msg_(msg)
  {}
  Init_FirmwareUpdateEthercat_Request_password file_path(::rokubimini_msgs::srv::FirmwareUpdateEthercat_Request::_file_path_type arg)
  {
    msg_.file_path = std::move(arg);
    return Init_FirmwareUpdateEthercat_Request_password(msg_);
  }

private:
  ::rokubimini_msgs::srv::FirmwareUpdateEthercat_Request msg_;
};

class Init_FirmwareUpdateEthercat_Request_file_name
{
public:
  Init_FirmwareUpdateEthercat_Request_file_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FirmwareUpdateEthercat_Request_file_path file_name(::rokubimini_msgs::srv::FirmwareUpdateEthercat_Request::_file_name_type arg)
  {
    msg_.file_name = std::move(arg);
    return Init_FirmwareUpdateEthercat_Request_file_path(msg_);
  }

private:
  ::rokubimini_msgs::srv::FirmwareUpdateEthercat_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rokubimini_msgs::srv::FirmwareUpdateEthercat_Request>()
{
  return rokubimini_msgs::srv::builder::Init_FirmwareUpdateEthercat_Request_file_name();
}

}  // namespace rokubimini_msgs


namespace rokubimini_msgs
{

namespace srv
{

namespace builder
{

class Init_FirmwareUpdateEthercat_Response_result
{
public:
  Init_FirmwareUpdateEthercat_Response_result()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rokubimini_msgs::srv::FirmwareUpdateEthercat_Response result(::rokubimini_msgs::srv::FirmwareUpdateEthercat_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rokubimini_msgs::srv::FirmwareUpdateEthercat_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rokubimini_msgs::srv::FirmwareUpdateEthercat_Response>()
{
  return rokubimini_msgs::srv::builder::Init_FirmwareUpdateEthercat_Response_result();
}

}  // namespace rokubimini_msgs

#endif  // ROKUBIMINI_MSGS__SRV__DETAIL__FIRMWARE_UPDATE_ETHERCAT__BUILDER_HPP_
