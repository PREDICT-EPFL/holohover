// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rokubimini_msgs:srv/ResetWrench.idl
// generated code does not contain a copyright notice

#ifndef ROKUBIMINI_MSGS__SRV__DETAIL__RESET_WRENCH__BUILDER_HPP_
#define ROKUBIMINI_MSGS__SRV__DETAIL__RESET_WRENCH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rokubimini_msgs/srv/detail/reset_wrench__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rokubimini_msgs
{

namespace srv
{

namespace builder
{

class Init_ResetWrench_Request_desired_wrench
{
public:
  Init_ResetWrench_Request_desired_wrench()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rokubimini_msgs::srv::ResetWrench_Request desired_wrench(::rokubimini_msgs::srv::ResetWrench_Request::_desired_wrench_type arg)
  {
    msg_.desired_wrench = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rokubimini_msgs::srv::ResetWrench_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rokubimini_msgs::srv::ResetWrench_Request>()
{
  return rokubimini_msgs::srv::builder::Init_ResetWrench_Request_desired_wrench();
}

}  // namespace rokubimini_msgs


namespace rokubimini_msgs
{

namespace srv
{

namespace builder
{

class Init_ResetWrench_Response_success
{
public:
  Init_ResetWrench_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rokubimini_msgs::srv::ResetWrench_Response success(::rokubimini_msgs::srv::ResetWrench_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rokubimini_msgs::srv::ResetWrench_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rokubimini_msgs::srv::ResetWrench_Response>()
{
  return rokubimini_msgs::srv::builder::Init_ResetWrench_Response_success();
}

}  // namespace rokubimini_msgs

#endif  // ROKUBIMINI_MSGS__SRV__DETAIL__RESET_WRENCH__BUILDER_HPP_
