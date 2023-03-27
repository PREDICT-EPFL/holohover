// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rokubimini_msgs:srv/FirmwareUpdateSerial.idl
// generated code does not contain a copyright notice

#ifndef ROKUBIMINI_MSGS__SRV__DETAIL__FIRMWARE_UPDATE_SERIAL__STRUCT_HPP_
#define ROKUBIMINI_MSGS__SRV__DETAIL__FIRMWARE_UPDATE_SERIAL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rokubimini_msgs__srv__FirmwareUpdateSerial_Request __attribute__((deprecated))
#else
# define DEPRECATED__rokubimini_msgs__srv__FirmwareUpdateSerial_Request __declspec(deprecated)
#endif

namespace rokubimini_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct FirmwareUpdateSerial_Request_
{
  using Type = FirmwareUpdateSerial_Request_<ContainerAllocator>;

  explicit FirmwareUpdateSerial_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->file_path = "";
    }
  }

  explicit FirmwareUpdateSerial_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : file_path(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->file_path = "";
    }
  }

  // field types and members
  using _file_path_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _file_path_type file_path;

  // setters for named parameter idiom
  Type & set__file_path(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->file_path = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rokubimini_msgs::srv::FirmwareUpdateSerial_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const rokubimini_msgs::srv::FirmwareUpdateSerial_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rokubimini_msgs::srv::FirmwareUpdateSerial_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rokubimini_msgs::srv::FirmwareUpdateSerial_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rokubimini_msgs::srv::FirmwareUpdateSerial_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rokubimini_msgs::srv::FirmwareUpdateSerial_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rokubimini_msgs::srv::FirmwareUpdateSerial_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rokubimini_msgs::srv::FirmwareUpdateSerial_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rokubimini_msgs::srv::FirmwareUpdateSerial_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rokubimini_msgs::srv::FirmwareUpdateSerial_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rokubimini_msgs__srv__FirmwareUpdateSerial_Request
    std::shared_ptr<rokubimini_msgs::srv::FirmwareUpdateSerial_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rokubimini_msgs__srv__FirmwareUpdateSerial_Request
    std::shared_ptr<rokubimini_msgs::srv::FirmwareUpdateSerial_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FirmwareUpdateSerial_Request_ & other) const
  {
    if (this->file_path != other.file_path) {
      return false;
    }
    return true;
  }
  bool operator!=(const FirmwareUpdateSerial_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FirmwareUpdateSerial_Request_

// alias to use template instance with default allocator
using FirmwareUpdateSerial_Request =
  rokubimini_msgs::srv::FirmwareUpdateSerial_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rokubimini_msgs


#ifndef _WIN32
# define DEPRECATED__rokubimini_msgs__srv__FirmwareUpdateSerial_Response __attribute__((deprecated))
#else
# define DEPRECATED__rokubimini_msgs__srv__FirmwareUpdateSerial_Response __declspec(deprecated)
#endif

namespace rokubimini_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct FirmwareUpdateSerial_Response_
{
  using Type = FirmwareUpdateSerial_Response_<ContainerAllocator>;

  explicit FirmwareUpdateSerial_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = false;
    }
  }

  explicit FirmwareUpdateSerial_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = false;
    }
  }

  // field types and members
  using _result_type =
    bool;
  _result_type result;

  // setters for named parameter idiom
  Type & set__result(
    const bool & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rokubimini_msgs::srv::FirmwareUpdateSerial_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const rokubimini_msgs::srv::FirmwareUpdateSerial_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rokubimini_msgs::srv::FirmwareUpdateSerial_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rokubimini_msgs::srv::FirmwareUpdateSerial_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rokubimini_msgs::srv::FirmwareUpdateSerial_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rokubimini_msgs::srv::FirmwareUpdateSerial_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rokubimini_msgs::srv::FirmwareUpdateSerial_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rokubimini_msgs::srv::FirmwareUpdateSerial_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rokubimini_msgs::srv::FirmwareUpdateSerial_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rokubimini_msgs::srv::FirmwareUpdateSerial_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rokubimini_msgs__srv__FirmwareUpdateSerial_Response
    std::shared_ptr<rokubimini_msgs::srv::FirmwareUpdateSerial_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rokubimini_msgs__srv__FirmwareUpdateSerial_Response
    std::shared_ptr<rokubimini_msgs::srv::FirmwareUpdateSerial_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FirmwareUpdateSerial_Response_ & other) const
  {
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const FirmwareUpdateSerial_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FirmwareUpdateSerial_Response_

// alias to use template instance with default allocator
using FirmwareUpdateSerial_Response =
  rokubimini_msgs::srv::FirmwareUpdateSerial_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rokubimini_msgs

namespace rokubimini_msgs
{

namespace srv
{

struct FirmwareUpdateSerial
{
  using Request = rokubimini_msgs::srv::FirmwareUpdateSerial_Request;
  using Response = rokubimini_msgs::srv::FirmwareUpdateSerial_Response;
};

}  // namespace srv

}  // namespace rokubimini_msgs

#endif  // ROKUBIMINI_MSGS__SRV__DETAIL__FIRMWARE_UPDATE_SERIAL__STRUCT_HPP_
