// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from nidar_msgs:msg/PersonDetection.idl
// generated code does not contain a copyright notice

#ifndef NIDAR_MSGS__MSG__DETAIL__PERSON_DETECTION__TRAITS_HPP_
#define NIDAR_MSGS__MSG__DETAIL__PERSON_DETECTION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "nidar_msgs/msg/detail/person_detection__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace nidar_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const PersonDetection & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: confidence
  {
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << ", ";
  }

  // member: x_min
  {
    out << "x_min: ";
    rosidl_generator_traits::value_to_yaml(msg.x_min, out);
    out << ", ";
  }

  // member: y_min
  {
    out << "y_min: ";
    rosidl_generator_traits::value_to_yaml(msg.y_min, out);
    out << ", ";
  }

  // member: x_max
  {
    out << "x_max: ";
    rosidl_generator_traits::value_to_yaml(msg.x_max, out);
    out << ", ";
  }

  // member: y_max
  {
    out << "y_max: ";
    rosidl_generator_traits::value_to_yaml(msg.y_max, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PersonDetection & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: confidence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << "\n";
  }

  // member: x_min
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x_min: ";
    rosidl_generator_traits::value_to_yaml(msg.x_min, out);
    out << "\n";
  }

  // member: y_min
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y_min: ";
    rosidl_generator_traits::value_to_yaml(msg.y_min, out);
    out << "\n";
  }

  // member: x_max
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x_max: ";
    rosidl_generator_traits::value_to_yaml(msg.x_max, out);
    out << "\n";
  }

  // member: y_max
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y_max: ";
    rosidl_generator_traits::value_to_yaml(msg.y_max, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PersonDetection & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace nidar_msgs

namespace rosidl_generator_traits
{

[[deprecated("use nidar_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const nidar_msgs::msg::PersonDetection & msg,
  std::ostream & out, size_t indentation = 0)
{
  nidar_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use nidar_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const nidar_msgs::msg::PersonDetection & msg)
{
  return nidar_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<nidar_msgs::msg::PersonDetection>()
{
  return "nidar_msgs::msg::PersonDetection";
}

template<>
inline const char * name<nidar_msgs::msg::PersonDetection>()
{
  return "nidar_msgs/msg/PersonDetection";
}

template<>
struct has_fixed_size<nidar_msgs::msg::PersonDetection>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<nidar_msgs::msg::PersonDetection>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<nidar_msgs::msg::PersonDetection>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // NIDAR_MSGS__MSG__DETAIL__PERSON_DETECTION__TRAITS_HPP_
