# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rokubimini_msgs:srv/SetSensorConfiguration.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SetSensorConfiguration_Request(type):
    """Metaclass of message 'SetSensorConfiguration_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('rokubimini_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'rokubimini_msgs.srv.SetSensorConfiguration_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_sensor_configuration__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_sensor_configuration__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_sensor_configuration__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_sensor_configuration__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_sensor_configuration__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SetSensorConfiguration_Request(metaclass=Metaclass_SetSensorConfiguration_Request):
    """Message class 'SetSensorConfiguration_Request'."""

    __slots__ = [
        '_a',
    ]

    _fields_and_field_types = {
        'a': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.a = kwargs.get('a', bool())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.a != other.a:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def a(self):
        """Message field 'a'."""
        return self._a

    @a.setter
    def a(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'a' field must be of type 'bool'"
        self._a = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_SetSensorConfiguration_Response(type):
    """Metaclass of message 'SetSensorConfiguration_Response'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('rokubimini_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'rokubimini_msgs.srv.SetSensorConfiguration_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_sensor_configuration__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_sensor_configuration__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_sensor_configuration__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_sensor_configuration__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_sensor_configuration__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SetSensorConfiguration_Response(metaclass=Metaclass_SetSensorConfiguration_Response):
    """Message class 'SetSensorConfiguration_Response'."""

    __slots__ = [
        '_b',
    ]

    _fields_and_field_types = {
        'b': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.b = kwargs.get('b', bool())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.b != other.b:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def b(self):
        """Message field 'b'."""
        return self._b

    @b.setter
    def b(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'b' field must be of type 'bool'"
        self._b = value


class Metaclass_SetSensorConfiguration(type):
    """Metaclass of service 'SetSensorConfiguration'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('rokubimini_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'rokubimini_msgs.srv.SetSensorConfiguration')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__set_sensor_configuration

            from rokubimini_msgs.srv import _set_sensor_configuration
            if _set_sensor_configuration.Metaclass_SetSensorConfiguration_Request._TYPE_SUPPORT is None:
                _set_sensor_configuration.Metaclass_SetSensorConfiguration_Request.__import_type_support__()
            if _set_sensor_configuration.Metaclass_SetSensorConfiguration_Response._TYPE_SUPPORT is None:
                _set_sensor_configuration.Metaclass_SetSensorConfiguration_Response.__import_type_support__()


class SetSensorConfiguration(metaclass=Metaclass_SetSensorConfiguration):
    from rokubimini_msgs.srv._set_sensor_configuration import SetSensorConfiguration_Request as Request
    from rokubimini_msgs.srv._set_sensor_configuration import SetSensorConfiguration_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
