# generated from rosidl_generator_py/resource/_idl.py.em
# with input from holohover_msgs:msg/HolohoverStateStamped.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_HolohoverStateStamped(type):
    """Metaclass of message 'HolohoverStateStamped'."""

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
            module = import_type_support('holohover_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'holohover_msgs.msg.HolohoverStateStamped')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__holohover_state_stamped
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__holohover_state_stamped
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__holohover_state_stamped
            cls._TYPE_SUPPORT = module.type_support_msg__msg__holohover_state_stamped
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__holohover_state_stamped

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class HolohoverStateStamped(metaclass=Metaclass_HolohoverStateStamped):
    """Message class 'HolohoverStateStamped'."""

    __slots__ = [
        '_header',
        '_x',
        '_y',
        '_v_x',
        '_v_y',
        '_yaw',
        '_w_z',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'x': 'double',
        'y': 'double',
        'v_x': 'double',
        'v_y': 'double',
        'yaw': 'double',
        'w_z': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.x = kwargs.get('x', float())
        self.y = kwargs.get('y', float())
        self.v_x = kwargs.get('v_x', float())
        self.v_y = kwargs.get('v_y', float())
        self.yaw = kwargs.get('yaw', float())
        self.w_z = kwargs.get('w_z', float())

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
        if self.header != other.header:
            return False
        if self.x != other.x:
            return False
        if self.y != other.y:
            return False
        if self.v_x != other.v_x:
            return False
        if self.v_y != other.v_y:
            return False
        if self.yaw != other.yaw:
            return False
        if self.w_z != other.w_z:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def x(self):
        """Message field 'x'."""
        return self._x

    @x.setter
    def x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._x = value

    @builtins.property
    def y(self):
        """Message field 'y'."""
        return self._y

    @y.setter
    def y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._y = value

    @builtins.property
    def v_x(self):
        """Message field 'v_x'."""
        return self._v_x

    @v_x.setter
    def v_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'v_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'v_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._v_x = value

    @builtins.property
    def v_y(self):
        """Message field 'v_y'."""
        return self._v_y

    @v_y.setter
    def v_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'v_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'v_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._v_y = value

    @builtins.property
    def yaw(self):
        """Message field 'yaw'."""
        return self._yaw

    @yaw.setter
    def yaw(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'yaw' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'yaw' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._yaw = value

    @builtins.property
    def w_z(self):
        """Message field 'w_z'."""
        return self._w_z

    @w_z.setter
    def w_z(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'w_z' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'w_z' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._w_z = value
