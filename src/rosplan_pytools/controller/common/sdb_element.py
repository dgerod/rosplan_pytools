import json
from rosplan_pytools.common import message_converter


class Element(object):

    @staticmethod
    def extract_ros_type(message):
        return message._type

    def __init__(self, value=None, metadata=""):

        self._metadata = metadata

        if value is not None:
            self._ros_message_value = value
            self._ros_message_type = self.extract_ros_type(value)
        else:
            self._ros_message_value = None
            self._ros_message_type = ""

    def __str__(self):

        if self._ros_message_value is not None:
            value = self._ros_message_value
        else:
            value = ""

        return "%s, %s, %s" % (self._ros_message_type, value, self._metadata)

    def __eq__(self, other):

        return self._ros_message_type == other._ros_message_type

    def clean(self):
        self._metadata = ""
        self._ros_message_value = None
        self._ros_message_type = ""

    def is_valid(self):
        return self._ros_message_value is not None

    def metadata(self):
        return self._metadata

    def value(self):
        return self._ros_message_value

    def type(self):
        return self._ros_message_type


def convert_element_to_string(element):

    if not isinstance(element, Element):
        raise TypeError

    dict_ = {'msg_type': element.type(),
             'msg_value': message_converter.convert_ros_message_to_dictionary(element.value())}

    value = json.dumps(dict_)
    return element.metadata(), value


def convert_string_to_element(metadata, value):

    if not isinstance(metadata, str) or not isinstance(value, str):
        raise TypeError

    dict_ = json.loads(value)
    msg_value = message_converter.convert_dictionary_to_ros_message(dict_['msg_type'],
                                                                    dict_['msg_value'],
                                                                    'message')
    return Element(msg_value, metadata)
