// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msg:srv/AddThreeInts.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSG__SRV__DETAIL__ADD_THREE_INTS__STRUCT_H_
#define CUSTOM_MSG__SRV__DETAIL__ADD_THREE_INTS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/AddThreeInts in the package custom_msg.
typedef struct custom_msg__srv__AddThreeInts_Request
{
  int64_t a;
  int64_t b;
  int64_t c;
} custom_msg__srv__AddThreeInts_Request;

// Struct for a sequence of custom_msg__srv__AddThreeInts_Request.
typedef struct custom_msg__srv__AddThreeInts_Request__Sequence
{
  custom_msg__srv__AddThreeInts_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msg__srv__AddThreeInts_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/AddThreeInts in the package custom_msg.
typedef struct custom_msg__srv__AddThreeInts_Response
{
  int64_t sum;
} custom_msg__srv__AddThreeInts_Response;

// Struct for a sequence of custom_msg__srv__AddThreeInts_Response.
typedef struct custom_msg__srv__AddThreeInts_Response__Sequence
{
  custom_msg__srv__AddThreeInts_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msg__srv__AddThreeInts_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSG__SRV__DETAIL__ADD_THREE_INTS__STRUCT_H_
