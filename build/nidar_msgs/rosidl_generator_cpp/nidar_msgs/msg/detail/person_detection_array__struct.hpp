// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from nidar_msgs:msg/PersonDetectionArray.idl
// generated code does not contain a copyright notice

#ifndef NIDAR_MSGS__MSG__DETAIL__PERSON_DETECTION_ARRAY__STRUCT_HPP_
#define NIDAR_MSGS__MSG__DETAIL__PERSON_DETECTION_ARRAY__STRUCT_HPP_

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
// Member 'detections'
#include "nidar_msgs/msg/detail/person_detection__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__nidar_msgs__msg__PersonDetectionArray __attribute__((deprecated))
#else
# define DEPRECATED__nidar_msgs__msg__PersonDetectionArray __declspec(deprecated)
#endif

namespace nidar_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PersonDetectionArray_
{
  using Type = PersonDetectionArray_<ContainerAllocator>;

  explicit PersonDetectionArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit PersonDetectionArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _detections_type =
    std::vector<nidar_msgs::msg::PersonDetection_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<nidar_msgs::msg::PersonDetection_<ContainerAllocator>>>;
  _detections_type detections;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__detections(
    const std::vector<nidar_msgs::msg::PersonDetection_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<nidar_msgs::msg::PersonDetection_<ContainerAllocator>>> & _arg)
  {
    this->detections = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    nidar_msgs::msg::PersonDetectionArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const nidar_msgs::msg::PersonDetectionArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<nidar_msgs::msg::PersonDetectionArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<nidar_msgs::msg::PersonDetectionArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      nidar_msgs::msg::PersonDetectionArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<nidar_msgs::msg::PersonDetectionArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      nidar_msgs::msg::PersonDetectionArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<nidar_msgs::msg::PersonDetectionArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<nidar_msgs::msg::PersonDetectionArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<nidar_msgs::msg::PersonDetectionArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__nidar_msgs__msg__PersonDetectionArray
    std::shared_ptr<nidar_msgs::msg::PersonDetectionArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__nidar_msgs__msg__PersonDetectionArray
    std::shared_ptr<nidar_msgs::msg::PersonDetectionArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PersonDetectionArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->detections != other.detections) {
      return false;
    }
    return true;
  }
  bool operator!=(const PersonDetectionArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PersonDetectionArray_

// alias to use template instance with default allocator
using PersonDetectionArray =
  nidar_msgs::msg::PersonDetectionArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace nidar_msgs

#endif  // NIDAR_MSGS__MSG__DETAIL__PERSON_DETECTION_ARRAY__STRUCT_HPP_
