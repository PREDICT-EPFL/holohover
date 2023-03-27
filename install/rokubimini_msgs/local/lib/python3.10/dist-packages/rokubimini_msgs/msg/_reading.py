# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rokubimini_msgs:msg/Reading.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Reading(type):
    """Metaclass of message 'Reading'."""

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
                'rokubimini_msgs.msg.Reading')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__reading
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__reading
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__reading
            cls._TYPE_SUPPORT = module.type_support_msg__msg__reading
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__reading

            from geometry_msgs.msg import WrenchStamped
            if WrenchStamped.__class__._TYPE_SUPPORT is None:
                WrenchStamped.__class__.__import_type_support__()

            from sensor_msgs.msg import Imu
            if Imu.__class__._TYPE_SUPPORT is None:
                Imu.__class__.__import_type_support__()

            from sensor_msgs.msg import Temperature
            if Temperature.__class__._TYPE_SUPPORT is None:
                Temperature.__class__.__import_type_support__()

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


class Reading(metaclass=Metaclass_Reading):
    """Message class 'Reading'."""

    __slots__ = [
        '_header',
        '_statusword',
        '_imu',
        '_wrench',
        '_external_imu',
        '_is_force_torque_saturated',
        '_temperature',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'statusword': 'uint32',
        'imu': 'sensor_msgs/Imu',
        'wrench': 'geometry_msgs/WrenchStamped',
        'external_imu': 'sensor_msgs/Imu',
        'is_force_torque_saturated': 'boolean',
        'temperature': 'sensor_msgs/Temperature',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'Imu'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'WrenchStamped'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'Imu'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'Temperature'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.statusword = kwargs.get('statusword', int())
        from sensor_msgs.msg import Imu
        self.imu = kwargs.get('imu', Imu())
        from geometry_msgs.msg import WrenchStamped
        self.wrench = kwargs.get('wrench', WrenchStamped())
        from sensor_msgs.msg import Imu
        self.external_imu = kwargs.get('external_imu', Imu())
        self.is_force_torque_saturated = kwargs.get('is_force_torque_saturated', bool())
        from sensor_msgs.msg import Temperature
        self.temperature = kwargs.get('temperature', Temperature())

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
        if self.statusword != other.statusword:
            return False
        if self.imu != other.imu:
            return False
        if self.wrench != other.wrench:
            return False
        if self.external_imu != other.external_imu:
            return False
        if self.is_force_torque_saturated != other.is_force_torque_saturated:
            return False
        if self.temperature != other.temperature:
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
    def statusword(self):
        """Message field 'statusword'."""
        return self._statusword

    @statusword.setter
    def statusword(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'statusword' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'statusword' field must be an unsigned integer in [0, 4294967295]"
        self._statusword = value

    @builtins.property
    def imu(self):
        """Message field 'imu'."""
        return self._imu

    @imu.setter
    def imu(self, value):
        if __debug__:
            from sensor_msgs.msg import Imu
            assert \
                isinstance(value, Imu), \
                "The 'imu' field must be a sub message of type 'Imu'"
        self._imu = value

    @builtins.property
    def wrench(self):
        """Message field 'wrench'."""
        return self._wrench

    @wrench.setter
    def wrench(self, value):
        if __debug__:
            from geometry_msgs.msg import WrenchStamped
            assert \
                isinstance(value, WrenchStamped), \
                "The 'wrench' field must be a sub message of type 'WrenchStamped'"
        self._wrench = value

    @builtins.property
    def external_imu(self):
        """Message field 'external_imu'."""
        return self._external_imu

    @external_imu.setter
    def external_imu(self, value):
        if __debug__:
            from sensor_msgs.msg import Imu
            assert \
                isinstance(value, Imu), \
                "The 'external_imu' field must be a sub message of type 'Imu'"
        self._external_imu = value

    @builtins.property
    def is_force_torque_saturated(self):
        """Message field 'is_force_torque_saturated'."""
        return self._is_force_torque_saturated

    @is_force_torque_saturated.setter
    def is_force_torque_saturated(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_force_torque_saturated' field must be of type 'bool'"
        self._is_force_torque_saturated = value

    @builtins.property
    def temperature(self):
        """Message field 'temperature'."""
        return self._temperature

    @temperature.setter
    def temperature(self, value):
        if __debug__:
            from sensor_msgs.msg import Temperature
            assert \
                isinstance(value, Temperature), \
                "The 'temperature' field must be a sub message of type 'Temperature'"
        self._temperature = value
