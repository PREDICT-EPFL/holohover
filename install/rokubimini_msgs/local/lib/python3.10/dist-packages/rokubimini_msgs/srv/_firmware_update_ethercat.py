# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rokubimini_msgs:srv/FirmwareUpdateEthercat.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_FirmwareUpdateEthercat_Request(type):
    """Metaclass of message 'FirmwareUpdateEthercat_Request'."""

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
                'rokubimini_msgs.srv.FirmwareUpdateEthercat_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__firmware_update_ethercat__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__firmware_update_ethercat__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__firmware_update_ethercat__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__firmware_update_ethercat__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__firmware_update_ethercat__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class FirmwareUpdateEthercat_Request(metaclass=Metaclass_FirmwareUpdateEthercat_Request):
    """Message class 'FirmwareUpdateEthercat_Request'."""

    __slots__ = [
        '_file_name',
        '_file_path',
        '_password',
    ]

    _fields_and_field_types = {
        'file_name': 'string',
        'file_path': 'string',
        'password': 'uint32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.file_name = kwargs.get('file_name', str())
        self.file_path = kwargs.get('file_path', str())
        self.password = kwargs.get('password', int())

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
        if self.file_name != other.file_name:
            return False
        if self.file_path != other.file_path:
            return False
        if self.password != other.password:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def file_name(self):
        """Message field 'file_name'."""
        return self._file_name

    @file_name.setter
    def file_name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'file_name' field must be of type 'str'"
        self._file_name = value

    @builtins.property
    def file_path(self):
        """Message field 'file_path'."""
        return self._file_path

    @file_path.setter
    def file_path(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'file_path' field must be of type 'str'"
        self._file_path = value

    @builtins.property
    def password(self):
        """Message field 'password'."""
        return self._password

    @password.setter
    def password(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'password' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'password' field must be an unsigned integer in [0, 4294967295]"
        self._password = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_FirmwareUpdateEthercat_Response(type):
    """Metaclass of message 'FirmwareUpdateEthercat_Response'."""

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
                'rokubimini_msgs.srv.FirmwareUpdateEthercat_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__firmware_update_ethercat__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__firmware_update_ethercat__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__firmware_update_ethercat__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__firmware_update_ethercat__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__firmware_update_ethercat__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class FirmwareUpdateEthercat_Response(metaclass=Metaclass_FirmwareUpdateEthercat_Response):
    """Message class 'FirmwareUpdateEthercat_Response'."""

    __slots__ = [
        '_result',
    ]

    _fields_and_field_types = {
        'result': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.result = kwargs.get('result', bool())

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
        if self.result != other.result:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def result(self):
        """Message field 'result'."""
        return self._result

    @result.setter
    def result(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'result' field must be of type 'bool'"
        self._result = value


class Metaclass_FirmwareUpdateEthercat(type):
    """Metaclass of service 'FirmwareUpdateEthercat'."""

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
                'rokubimini_msgs.srv.FirmwareUpdateEthercat')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__firmware_update_ethercat

            from rokubimini_msgs.srv import _firmware_update_ethercat
            if _firmware_update_ethercat.Metaclass_FirmwareUpdateEthercat_Request._TYPE_SUPPORT is None:
                _firmware_update_ethercat.Metaclass_FirmwareUpdateEthercat_Request.__import_type_support__()
            if _firmware_update_ethercat.Metaclass_FirmwareUpdateEthercat_Response._TYPE_SUPPORT is None:
                _firmware_update_ethercat.Metaclass_FirmwareUpdateEthercat_Response.__import_type_support__()


class FirmwareUpdateEthercat(metaclass=Metaclass_FirmwareUpdateEthercat):
    from rokubimini_msgs.srv._firmware_update_ethercat import FirmwareUpdateEthercat_Request as Request
    from rokubimini_msgs.srv._firmware_update_ethercat import FirmwareUpdateEthercat_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
