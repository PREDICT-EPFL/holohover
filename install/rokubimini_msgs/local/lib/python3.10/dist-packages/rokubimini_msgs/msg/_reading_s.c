// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from rokubimini_msgs:msg/Reading.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "rokubimini_msgs/msg/detail/reading__struct.h"
#include "rokubimini_msgs/msg/detail/reading__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool sensor_msgs__msg__imu__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * sensor_msgs__msg__imu__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__wrench_stamped__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__wrench_stamped__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool sensor_msgs__msg__imu__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * sensor_msgs__msg__imu__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool sensor_msgs__msg__temperature__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * sensor_msgs__msg__temperature__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool rokubimini_msgs__msg__reading__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[37];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("rokubimini_msgs.msg._reading.Reading", full_classname_dest, 36) == 0);
  }
  rokubimini_msgs__msg__Reading * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // statusword
    PyObject * field = PyObject_GetAttrString(_pymsg, "statusword");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->statusword = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // imu
    PyObject * field = PyObject_GetAttrString(_pymsg, "imu");
    if (!field) {
      return false;
    }
    if (!sensor_msgs__msg__imu__convert_from_py(field, &ros_message->imu)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // wrench
    PyObject * field = PyObject_GetAttrString(_pymsg, "wrench");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__wrench_stamped__convert_from_py(field, &ros_message->wrench)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // external_imu
    PyObject * field = PyObject_GetAttrString(_pymsg, "external_imu");
    if (!field) {
      return false;
    }
    if (!sensor_msgs__msg__imu__convert_from_py(field, &ros_message->external_imu)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // is_force_torque_saturated
    PyObject * field = PyObject_GetAttrString(_pymsg, "is_force_torque_saturated");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->is_force_torque_saturated = (Py_True == field);
    Py_DECREF(field);
  }
  {  // temperature
    PyObject * field = PyObject_GetAttrString(_pymsg, "temperature");
    if (!field) {
      return false;
    }
    if (!sensor_msgs__msg__temperature__convert_from_py(field, &ros_message->temperature)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * rokubimini_msgs__msg__reading__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Reading */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("rokubimini_msgs.msg._reading");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Reading");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  rokubimini_msgs__msg__Reading * ros_message = (rokubimini_msgs__msg__Reading *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // statusword
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->statusword);
    {
      int rc = PyObject_SetAttrString(_pymessage, "statusword", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // imu
    PyObject * field = NULL;
    field = sensor_msgs__msg__imu__convert_to_py(&ros_message->imu);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "imu", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // wrench
    PyObject * field = NULL;
    field = geometry_msgs__msg__wrench_stamped__convert_to_py(&ros_message->wrench);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "wrench", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // external_imu
    PyObject * field = NULL;
    field = sensor_msgs__msg__imu__convert_to_py(&ros_message->external_imu);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "external_imu", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // is_force_torque_saturated
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->is_force_torque_saturated ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "is_force_torque_saturated", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // temperature
    PyObject * field = NULL;
    field = sensor_msgs__msg__temperature__convert_to_py(&ros_message->temperature);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "temperature", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
