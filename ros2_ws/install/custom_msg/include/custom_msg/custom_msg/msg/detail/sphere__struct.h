// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msg:msg/Sphere.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSG__MSG__DETAIL__SPHERE__STRUCT_H_
#define CUSTOM_MSG__MSG__DETAIL__SPHERE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'cmd'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Sphere in the package custom_msg.
typedef struct custom_msg__msg__Sphere
{
  rosidl_runtime_c__String cmd;
  double latitude;
  double longitude;
} custom_msg__msg__Sphere;

// Struct for a sequence of custom_msg__msg__Sphere.
typedef struct custom_msg__msg__Sphere__Sequence
{
  custom_msg__msg__Sphere * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msg__msg__Sphere__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSG__MSG__DETAIL__SPHERE__STRUCT_H_
