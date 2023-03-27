// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rokubimini_msgs:srv/GetSensorConfiguration.idl
// generated code does not contain a copyright notice

#ifndef ROKUBIMINI_MSGS__SRV__DETAIL__GET_SENSOR_CONFIGURATION__STRUCT_HPP_
#define ROKUBIMINI_MSGS__SRV__DETAIL__GET_SENSOR_CONFIGURATION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rokubimini_msgs__srv__GetSensorConfiguration_Request __attribute__((deprecated))
#else
# define DEPRECATED__rokubimini_msgs__srv__GetSensorConfiguration_Request __declspec(deprecated)
#endif

namespace rokubimini_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetSensorConfiguration_Request_
{
  using Type = GetSensorConfiguration_Request_<ContainerAllocator>;

  explicit GetSensorConfiguration_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->a = false;
    }
  }

  explicit GetSensorConfiguration_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->a = false;
    }
  }

  // field types and members
  using _a_type =
    bool;
  _a_type a;

  // setters for named parameter idiom
  Type & set__a(
    const bool & _arg)
  {
    this->a = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rokubimini_msgs::srv::GetSensorConfiguration_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const rokubimini_msgs::srv::GetSensorConfiguration_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rokubimini_msgs::srv::GetSensorConfiguration_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rokubimini_msgs::srv::GetSensorConfiguration_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rokubimini_msgs::srv::GetSensorConfiguration_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rokubimini_msgs::srv::GetSensorConfiguration_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rokubimini_msgs::srv::GetSensorConfiguration_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rokubimini_msgs::srv::GetSensorConfiguration_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rokubimini_msgs::srv::GetSensorConfiguration_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rokubimini_msgs::srv::GetSensorConfiguration_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rokubimini_msgs__srv__GetSensorConfiguration_Request
    std::shared_ptr<rokubimini_msgs::srv::GetSensorConfiguration_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rokubimini_msgs__srv__GetSensorConfiguration_Request
    std::shared_ptr<rokubimini_msgs::srv::GetSensorConfiguration_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetSensorConfiguration_Request_ & other) const
  {
    if (this->a != other.a) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetSensorConfiguration_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetSensorConfiguration_Request_

// alias to use template instance with default allocator
using GetSensorConfiguration_Request =
  rokubimini_msgs::srv::GetSensorConfiguration_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rokubimini_msgs


#ifndef _WIN32
# define DEPRECATED__rokubimini_msgs__srv__GetSensorConfiguration_Response __attribute__((deprecated))
#else
# define DEPRECATED__rokubimini_msgs__srv__GetSensorConfiguration_Response __declspec(deprecated)
#endif

namespace rokubimini_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetSensorConfiguration_Response_
{
  using Type = GetSensorConfiguration_Response_<ContainerAllocator>;

  explicit GetSensorConfiguration_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->b = false;
    }
  }

  explicit GetSensorConfiguration_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->b = false;
    }
  }

  // field types and members
  using _b_type =
    bool;
  _b_type b;

  // setters for named parameter idiom
  Type & set__b(
    const bool & _arg)
  {
    this->b = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rokubimini_msgs::srv::GetSensorConfiguration_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const rokubimini_msgs::srv::GetSensorConfiguration_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rokubimini_msgs::srv::GetSensorConfiguration_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rokubimini_msgs::srv::GetSensorConfiguration_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rokubimini_msgs::srv::GetSensorConfiguration_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rokubimini_msgs::srv::GetSensorConfiguration_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rokubimini_msgs::srv::GetSensorConfiguration_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rokubimini_msgs::srv::GetSensorConfiguration_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rokubimini_msgs::srv::GetSensorConfiguration_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rokubimini_msgs::srv::GetSensorConfiguration_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rokubimini_msgs__srv__GetSensorConfiguration_Response
    std::shared_ptr<rokubimini_msgs::srv::GetSensorConfiguration_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rokubimini_msgs__srv__GetSensorConfiguration_Response
    std::shared_ptr<rokubimini_msgs::srv::GetSensorConfiguration_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetSensorConfiguration_Response_ & other) const
  {
    if (this->b != other.b) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetSensorConfiguration_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetSensorConfiguration_Response_

// alias to use template instance with default allocator
using GetSensorConfiguration_Response =
  rokubimini_msgs::srv::GetSensorConfiguration_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rokubimini_msgs

namespace rokubimini_msgs
{

namespace srv
{

struct GetSensorConfiguration
{
  using Request = rokubimini_msgs::srv::GetSensorConfiguration_Request;
  using Response = rokubimini_msgs::srv::GetSensorConfiguration_Response;
};

}  // namespace srv

}  // namespace rokubimini_msgs

#endif  // ROKUBIMINI_MSGS__SRV__DETAIL__GET_SENSOR_CONFIGURATION__STRUCT_HPP_
