// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rokubimini_msgs:srv/FirmwareUpdateEthercat.idl
// generated code does not contain a copyright notice

#ifndef ROKUBIMINI_MSGS__SRV__DETAIL__FIRMWARE_UPDATE_ETHERCAT__STRUCT_HPP_
#define ROKUBIMINI_MSGS__SRV__DETAIL__FIRMWARE_UPDATE_ETHERCAT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rokubimini_msgs__srv__FirmwareUpdateEthercat_Request __attribute__((deprecated))
#else
# define DEPRECATED__rokubimini_msgs__srv__FirmwareUpdateEthercat_Request __declspec(deprecated)
#endif

namespace rokubimini_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct FirmwareUpdateEthercat_Request_
{
  using Type = FirmwareUpdateEthercat_Request_<ContainerAllocator>;

  explicit FirmwareUpdateEthercat_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->file_name = "";
      this->file_path = "";
      this->password = 0ul;
    }
  }

  explicit FirmwareUpdateEthercat_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : file_name(_alloc),
    file_path(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->file_name = "";
      this->file_path = "";
      this->password = 0ul;
    }
  }

  // field types and members
  using _file_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _file_name_type file_name;
  using _file_path_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _file_path_type file_path;
  using _password_type =
    uint32_t;
  _password_type password;

  // setters for named parameter idiom
  Type & set__file_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->file_name = _arg;
    return *this;
  }
  Type & set__file_path(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->file_path = _arg;
    return *this;
  }
  Type & set__password(
    const uint32_t & _arg)
  {
    this->password = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rokubimini_msgs::srv::FirmwareUpdateEthercat_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const rokubimini_msgs::srv::FirmwareUpdateEthercat_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rokubimini_msgs::srv::FirmwareUpdateEthercat_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rokubimini_msgs::srv::FirmwareUpdateEthercat_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rokubimini_msgs::srv::FirmwareUpdateEthercat_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rokubimini_msgs::srv::FirmwareUpdateEthercat_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rokubimini_msgs::srv::FirmwareUpdateEthercat_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rokubimini_msgs::srv::FirmwareUpdateEthercat_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rokubimini_msgs::srv::FirmwareUpdateEthercat_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rokubimini_msgs::srv::FirmwareUpdateEthercat_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rokubimini_msgs__srv__FirmwareUpdateEthercat_Request
    std::shared_ptr<rokubimini_msgs::srv::FirmwareUpdateEthercat_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rokubimini_msgs__srv__FirmwareUpdateEthercat_Request
    std::shared_ptr<rokubimini_msgs::srv::FirmwareUpdateEthercat_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FirmwareUpdateEthercat_Request_ & other) const
  {
    if (this->file_name != other.file_name) {
      return false;
    }
    if (this->file_path != other.file_path) {
      return false;
    }
    if (this->password != other.password) {
      return false;
    }
    return true;
  }
  bool operator!=(const FirmwareUpdateEthercat_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FirmwareUpdateEthercat_Request_

// alias to use template instance with default allocator
using FirmwareUpdateEthercat_Request =
  rokubimini_msgs::srv::FirmwareUpdateEthercat_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rokubimini_msgs


#ifndef _WIN32
# define DEPRECATED__rokubimini_msgs__srv__FirmwareUpdateEthercat_Response __attribute__((deprecated))
#else
# define DEPRECATED__rokubimini_msgs__srv__FirmwareUpdateEthercat_Response __declspec(deprecated)
#endif

namespace rokubimini_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct FirmwareUpdateEthercat_Response_
{
  using Type = FirmwareUpdateEthercat_Response_<ContainerAllocator>;

  explicit FirmwareUpdateEthercat_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = false;
    }
  }

  explicit FirmwareUpdateEthercat_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    rokubimini_msgs::srv::FirmwareUpdateEthercat_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const rokubimini_msgs::srv::FirmwareUpdateEthercat_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rokubimini_msgs::srv::FirmwareUpdateEthercat_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rokubimini_msgs::srv::FirmwareUpdateEthercat_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rokubimini_msgs::srv::FirmwareUpdateEthercat_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rokubimini_msgs::srv::FirmwareUpdateEthercat_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rokubimini_msgs::srv::FirmwareUpdateEthercat_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rokubimini_msgs::srv::FirmwareUpdateEthercat_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rokubimini_msgs::srv::FirmwareUpdateEthercat_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rokubimini_msgs::srv::FirmwareUpdateEthercat_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rokubimini_msgs__srv__FirmwareUpdateEthercat_Response
    std::shared_ptr<rokubimini_msgs::srv::FirmwareUpdateEthercat_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rokubimini_msgs__srv__FirmwareUpdateEthercat_Response
    std::shared_ptr<rokubimini_msgs::srv::FirmwareUpdateEthercat_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FirmwareUpdateEthercat_Response_ & other) const
  {
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const FirmwareUpdateEthercat_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FirmwareUpdateEthercat_Response_

// alias to use template instance with default allocator
using FirmwareUpdateEthercat_Response =
  rokubimini_msgs::srv::FirmwareUpdateEthercat_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rokubimini_msgs

namespace rokubimini_msgs
{

namespace srv
{

struct FirmwareUpdateEthercat
{
  using Request = rokubimini_msgs::srv::FirmwareUpdateEthercat_Request;
  using Response = rokubimini_msgs::srv::FirmwareUpdateEthercat_Response;
};

}  // namespace srv

}  // namespace rokubimini_msgs

#endif  // ROKUBIMINI_MSGS__SRV__DETAIL__FIRMWARE_UPDATE_ETHERCAT__STRUCT_HPP_
