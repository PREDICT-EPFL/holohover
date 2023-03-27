// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rokubimini_msgs:srv/SetSensorConfiguration.idl
// generated code does not contain a copyright notice

#ifndef ROKUBIMINI_MSGS__SRV__DETAIL__SET_SENSOR_CONFIGURATION__BUILDER_HPP_
#define ROKUBIMINI_MSGS__SRV__DETAIL__SET_SENSOR_CONFIGURATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rokubimini_msgs/srv/detail/set_sensor_configuration__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rokubimini_msgs
{

namespace srv
{

namespace builder
{

class Init_SetSensorConfiguration_Request_a
{
public:
  Init_SetSensorConfiguration_Request_a()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rokubimini_msgs::srv::SetSensorConfiguration_Request a(::rokubimini_msgs::srv::SetSensorConfiguration_Request::_a_type arg)
  {
    msg_.a = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rokubimini_msgs::srv::SetSensorConfiguration_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rokubimini_msgs::srv::SetSensorConfiguration_Request>()
{
  return rokubimini_msgs::srv::builder::Init_SetSensorConfiguration_Request_a();
}

}  // namespace rokubimini_msgs


namespace rokubimini_msgs
{

namespace srv
{

namespace builder
{

class Init_SetSensorConfiguration_Response_b
{
public:
  Init_SetSensorConfiguration_Response_b()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rokubimini_msgs::srv::SetSensorConfiguration_Response b(::rokubimini_msgs::srv::SetSensorConfiguration_Response::_b_type arg)
  {
    msg_.b = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rokubimini_msgs::srv::SetSensorConfiguration_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rokubimini_msgs::srv::SetSensorConfiguration_Response>()
{
  return rokubimini_msgs::srv::builder::Init_SetSensorConfiguration_Response_b();
}

}  // namespace rokubimini_msgs

#endif  // ROKUBIMINI_MSGS__SRV__DETAIL__SET_SENSOR_CONFIGURATION__BUILDER_HPP_
