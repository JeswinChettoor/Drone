// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from nidar_msgs:msg/PersonDetection.idl
// generated code does not contain a copyright notice

#ifndef NIDAR_MSGS__MSG__DETAIL__PERSON_DETECTION__STRUCT_HPP_
#define NIDAR_MSGS__MSG__DETAIL__PERSON_DETECTION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__nidar_msgs__msg__PersonDetection __attribute__((deprecated))
#else
# define DEPRECATED__nidar_msgs__msg__PersonDetection __declspec(deprecated)
#endif

namespace nidar_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PersonDetection_
{
  using Type = PersonDetection_<ContainerAllocator>;

  explicit PersonDetection_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->confidence = 0.0f;
      this->x_min = 0.0f;
      this->y_min = 0.0f;
      this->x_max = 0.0f;
      this->y_max = 0.0f;
    }
  }

  explicit PersonDetection_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->confidence = 0.0f;
      this->x_min = 0.0f;
      this->y_min = 0.0f;
      this->x_max = 0.0f;
      this->y_max = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _confidence_type =
    float;
  _confidence_type confidence;
  using _x_min_type =
    float;
  _x_min_type x_min;
  using _y_min_type =
    float;
  _y_min_type y_min;
  using _x_max_type =
    float;
  _x_max_type x_max;
  using _y_max_type =
    float;
  _y_max_type y_max;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__confidence(
    const float & _arg)
  {
    this->confidence = _arg;
    return *this;
  }
  Type & set__x_min(
    const float & _arg)
  {
    this->x_min = _arg;
    return *this;
  }
  Type & set__y_min(
    const float & _arg)
  {
    this->y_min = _arg;
    return *this;
  }
  Type & set__x_max(
    const float & _arg)
  {
    this->x_max = _arg;
    return *this;
  }
  Type & set__y_max(
    const float & _arg)
  {
    this->y_max = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    nidar_msgs::msg::PersonDetection_<ContainerAllocator> *;
  using ConstRawPtr =
    const nidar_msgs::msg::PersonDetection_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<nidar_msgs::msg::PersonDetection_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<nidar_msgs::msg::PersonDetection_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      nidar_msgs::msg::PersonDetection_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<nidar_msgs::msg::PersonDetection_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      nidar_msgs::msg::PersonDetection_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<nidar_msgs::msg::PersonDetection_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<nidar_msgs::msg::PersonDetection_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<nidar_msgs::msg::PersonDetection_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__nidar_msgs__msg__PersonDetection
    std::shared_ptr<nidar_msgs::msg::PersonDetection_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__nidar_msgs__msg__PersonDetection
    std::shared_ptr<nidar_msgs::msg::PersonDetection_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PersonDetection_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->confidence != other.confidence) {
      return false;
    }
    if (this->x_min != other.x_min) {
      return false;
    }
    if (this->y_min != other.y_min) {
      return false;
    }
    if (this->x_max != other.x_max) {
      return false;
    }
    if (this->y_max != other.y_max) {
      return false;
    }
    return true;
  }
  bool operator!=(const PersonDetection_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PersonDetection_

// alias to use template instance with default allocator
using PersonDetection =
  nidar_msgs::msg::PersonDetection_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace nidar_msgs

#endif  // NIDAR_MSGS__MSG__DETAIL__PERSON_DETECTION__STRUCT_HPP_
