import json
from rosplan_pytools.common import message_converter
from rosplan_pytools.controller.common.sdb_element import Element


def sdb_element_to_string(element):

    if not isinstance(element, Element):
        raise TypeError

    dict_ = {'msg_type': element.type(),
             'msg_value': message_converter.convert_ros_message_to_dictionary(element.value())}

    value = json.dumps(dict_)
    return element.metadata(), value


def string_to_sdb_element(metadata, value):

    if not isinstance(metadata, str) or not isinstance(value, str):
        raise TypeError

    dict_ = json.loads(value)
    msg_value = message_converter.convert_dictionary_to_ros_message(dict_['msg_type'],
                                                                    dict_['msg_value'],
                                                                    'message')
    return Element(msg_value, metadata)
