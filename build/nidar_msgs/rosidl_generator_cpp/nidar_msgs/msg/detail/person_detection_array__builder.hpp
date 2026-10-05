// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from nidar_msgs:msg/PersonDetectionArray.idl
// generated code does not contain a copyright notice

#ifndef NIDAR_MSGS__MSG__DETAIL__PERSON_DETECTION_ARRAY__BUILDER_HPP_
#define NIDAR_MSGS__MSG__DETAIL__PERSON_DETECTION_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "nidar_msgs/msg/detail/person_detection_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace nidar_msgs
{

namespace msg
{

namespace builder
{

class Init_PersonDetectionArray_detections
{
public:
  explicit Init_PersonDetectionArray_detections(::nidar_msgs::msg::PersonDetectionArray & msg)
  : msg_(msg)
  {}
  ::nidar_msgs::msg::PersonDetectionArray detections(::nidar_msgs::msg::PersonDetectionArray::_detections_type arg)
  {
    msg_.detections = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nidar_msgs::msg::PersonDetectionArray msg_;
};

class Init_PersonDetectionArray_header
{
public:
  Init_PersonDetectionArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PersonDetectionArray_detections header(::nidar_msgs::msg::PersonDetectionArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PersonDetectionArray_detections(msg_);
  }

private:
  ::nidar_msgs::msg::PersonDetectionArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::nidar_msgs::msg::PersonDetectionArray>()
{
  return nidar_msgs::msg::builder::Init_PersonDetectionArray_header();
}

}  // namespace nidar_msgs

#endif  // NIDAR_MSGS__MSG__DETAIL__PERSON_DETECTION_ARRAY__BUILDER_HPP_
