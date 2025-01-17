// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msg:msg/Sphere.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSG__MSG__DETAIL__SPHERE__BUILDER_HPP_
#define CUSTOM_MSG__MSG__DETAIL__SPHERE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msg/msg/detail/sphere__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msg
{

namespace msg
{

namespace builder
{

class Init_Sphere_longitude
{
public:
  explicit Init_Sphere_longitude(::custom_msg::msg::Sphere & msg)
  : msg_(msg)
  {}
  ::custom_msg::msg::Sphere longitude(::custom_msg::msg::Sphere::_longitude_type arg)
  {
    msg_.longitude = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msg::msg::Sphere msg_;
};

class Init_Sphere_latitude
{
public:
  explicit Init_Sphere_latitude(::custom_msg::msg::Sphere & msg)
  : msg_(msg)
  {}
  Init_Sphere_longitude latitude(::custom_msg::msg::Sphere::_latitude_type arg)
  {
    msg_.latitude = std::move(arg);
    return Init_Sphere_longitude(msg_);
  }

private:
  ::custom_msg::msg::Sphere msg_;
};

class Init_Sphere_cmd
{
public:
  Init_Sphere_cmd()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Sphere_latitude cmd(::custom_msg::msg::Sphere::_cmd_type arg)
  {
    msg_.cmd = std::move(arg);
    return Init_Sphere_latitude(msg_);
  }

private:
  ::custom_msg::msg::Sphere msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msg::msg::Sphere>()
{
  return custom_msg::msg::builder::Init_Sphere_cmd();
}

}  // namespace custom_msg

#endif  // CUSTOM_MSG__MSG__DETAIL__SPHERE__BUILDER_HPP_
