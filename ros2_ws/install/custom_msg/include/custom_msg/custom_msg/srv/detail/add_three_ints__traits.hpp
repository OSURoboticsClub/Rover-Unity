// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_msg:srv/AddThreeInts.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSG__SRV__DETAIL__ADD_THREE_INTS__TRAITS_HPP_
#define CUSTOM_MSG__SRV__DETAIL__ADD_THREE_INTS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_msg/srv/detail/add_three_ints__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace custom_msg
{

namespace srv
{

inline void to_flow_style_yaml(
  const AddThreeInts_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: a
  {
    out << "a: ";
    rosidl_generator_traits::value_to_yaml(msg.a, out);
    out << ", ";
  }

  // member: b
  {
    out << "b: ";
    rosidl_generator_traits::value_to_yaml(msg.b, out);
    out << ", ";
  }

  // member: c
  {
    out << "c: ";
    rosidl_generator_traits::value_to_yaml(msg.c, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AddThreeInts_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: a
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "a: ";
    rosidl_generator_traits::value_to_yaml(msg.a, out);
    out << "\n";
  }

  // member: b
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "b: ";
    rosidl_generator_traits::value_to_yaml(msg.b, out);
    out << "\n";
  }

  // member: c
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "c: ";
    rosidl_generator_traits::value_to_yaml(msg.c, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AddThreeInts_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace custom_msg

namespace rosidl_generator_traits
{

[[deprecated("use custom_msg::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_msg::srv::AddThreeInts_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_msg::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_msg::srv::to_yaml() instead")]]
inline std::string to_yaml(const custom_msg::srv::AddThreeInts_Request & msg)
{
  return custom_msg::srv::to_yaml(msg);
}

template<>
inline const char * data_type<custom_msg::srv::AddThreeInts_Request>()
{
  return "custom_msg::srv::AddThreeInts_Request";
}

template<>
inline const char * name<custom_msg::srv::AddThreeInts_Request>()
{
  return "custom_msg/srv/AddThreeInts_Request";
}

template<>
struct has_fixed_size<custom_msg::srv::AddThreeInts_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<custom_msg::srv::AddThreeInts_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<custom_msg::srv::AddThreeInts_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace custom_msg
{

namespace srv
{

inline void to_flow_style_yaml(
  const AddThreeInts_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: sum
  {
    out << "sum: ";
    rosidl_generator_traits::value_to_yaml(msg.sum, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AddThreeInts_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: sum
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sum: ";
    rosidl_generator_traits::value_to_yaml(msg.sum, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AddThreeInts_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace custom_msg

namespace rosidl_generator_traits
{

[[deprecated("use custom_msg::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_msg::srv::AddThreeInts_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_msg::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_msg::srv::to_yaml() instead")]]
inline std::string to_yaml(const custom_msg::srv::AddThreeInts_Response & msg)
{
  return custom_msg::srv::to_yaml(msg);
}

template<>
inline const char * data_type<custom_msg::srv::AddThreeInts_Response>()
{
  return "custom_msg::srv::AddThreeInts_Response";
}

template<>
inline const char * name<custom_msg::srv::AddThreeInts_Response>()
{
  return "custom_msg/srv/AddThreeInts_Response";
}

template<>
struct has_fixed_size<custom_msg::srv::AddThreeInts_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<custom_msg::srv::AddThreeInts_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<custom_msg::srv::AddThreeInts_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<custom_msg::srv::AddThreeInts>()
{
  return "custom_msg::srv::AddThreeInts";
}

template<>
inline const char * name<custom_msg::srv::AddThreeInts>()
{
  return "custom_msg/srv/AddThreeInts";
}

template<>
struct has_fixed_size<custom_msg::srv::AddThreeInts>
  : std::integral_constant<
    bool,
    has_fixed_size<custom_msg::srv::AddThreeInts_Request>::value &&
    has_fixed_size<custom_msg::srv::AddThreeInts_Response>::value
  >
{
};

template<>
struct has_bounded_size<custom_msg::srv::AddThreeInts>
  : std::integral_constant<
    bool,
    has_bounded_size<custom_msg::srv::AddThreeInts_Request>::value &&
    has_bounded_size<custom_msg::srv::AddThreeInts_Response>::value
  >
{
};

template<>
struct is_service<custom_msg::srv::AddThreeInts>
  : std::true_type
{
};

template<>
struct is_service_request<custom_msg::srv::AddThreeInts_Request>
  : std::true_type
{
};

template<>
struct is_service_response<custom_msg::srv::AddThreeInts_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_MSG__SRV__DETAIL__ADD_THREE_INTS__TRAITS_HPP_
