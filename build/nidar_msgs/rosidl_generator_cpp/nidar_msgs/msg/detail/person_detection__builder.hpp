// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from nidar_msgs:msg/PersonDetection.idl
// generated code does not contain a copyright notice

#ifndef NIDAR_MSGS__MSG__DETAIL__PERSON_DETECTION__BUILDER_HPP_
#define NIDAR_MSGS__MSG__DETAIL__PERSON_DETECTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "nidar_msgs/msg/detail/person_detection__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace nidar_msgs
{

namespace msg
{

namespace builder
{

class Init_PersonDetection_y_max
{
public:
  explicit Init_PersonDetection_y_max(::nidar_msgs::msg::PersonDetection & msg)
  : msg_(msg)
  {}
  ::nidar_msgs::msg::PersonDetection y_max(::nidar_msgs::msg::PersonDetection::_y_max_type arg)
  {
    msg_.y_max = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nidar_msgs::msg::PersonDetection msg_;
};

class Init_PersonDetection_x_max
{
public:
  explicit Init_PersonDetection_x_max(::nidar_msgs::msg::PersonDetection & msg)
  : msg_(msg)
  {}
  Init_PersonDetection_y_max x_max(::nidar_msgs::msg::PersonDetection::_x_max_type arg)
  {
    msg_.x_max = std::move(arg);
    return Init_PersonDetection_y_max(msg_);
  }

private:
  ::nidar_msgs::msg::PersonDetection msg_;
};

class Init_PersonDetection_y_min
{
public:
  explicit Init_PersonDetection_y_min(::nidar_msgs::msg::PersonDetection & msg)
  : msg_(msg)
  {}
  Init_PersonDetection_x_max y_min(::nidar_msgs::msg::PersonDetection::_y_min_type arg)
  {
    msg_.y_min = std::move(arg);
    return Init_PersonDetection_x_max(msg_);
  }

private:
  ::nidar_msgs::msg::PersonDetection msg_;
};

class Init_PersonDetection_x_min
{
public:
  explicit Init_PersonDetection_x_min(::nidar_msgs::msg::PersonDetection & msg)
  : msg_(msg)
  {}
  Init_PersonDetection_y_min x_min(::nidar_msgs::msg::PersonDetection::_x_min_type arg)
  {
    msg_.x_min = std::move(arg);
    return Init_PersonDetection_y_min(msg_);
  }

private:
  ::nidar_msgs::msg::PersonDetection msg_;
};

class Init_PersonDetection_confidence
{
public:
  explicit Init_PersonDetection_confidence(::nidar_msgs::msg::PersonDetection & msg)
  : msg_(msg)
  {}
  Init_PersonDetection_x_min confidence(::nidar_msgs::msg::PersonDetection::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_PersonDetection_x_min(msg_);
  }

private:
  ::nidar_msgs::msg::PersonDetection msg_;
};

class Init_PersonDetection_header
{
public:
  Init_PersonDetection_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PersonDetection_confidence header(::nidar_msgs::msg::PersonDetection::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PersonDetection_confidence(msg_);
  }

private:
  ::nidar_msgs::msg::PersonDetection msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::nidar_msgs::msg::PersonDetection>()
{
  return nidar_msgs::msg::builder::Init_PersonDetection_header();
}

}  // namespace nidar_msgs

#endif  // NIDAR_MSGS__MSG__DETAIL__PERSON_DETECTION__BUILDER_HPP_
