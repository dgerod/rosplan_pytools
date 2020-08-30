import json

from rosplan_pytools.rosparam.common.element import StorageElement
from rosplan_pytools.rosparam.common.ros_message_converter import convert_ros_message_to_dictionary, \
    convert_dictionary_to_ros_message


def element_to_data(element):
    # type: (StorageElement) -> str

    if not isinstance(element, StorageElement):
        raise TypeError

    dict_ = {'msg_type': element.type(),
             'msg_value': convert_ros_message_to_dictionary(element.value()),
             'metadata': element.metadata()}

    return json.dumps(dict_)


def data_to_element(data):
    # type: (str) -> StorageElement

    if not isinstance(data, str):
        raise TypeError

    dict_ = json.loads(data)
    message = convert_dictionary_to_ros_message(dict_['msg_type'],  dict_['msg_value'], 'message')

    return StorageElement(message, dict_['metadata'])


# def _replace_types_in_dictionary(dictionary):
#
#     for k, v in dictionary.iteritems():
#         if type(v) is dict:
#             replace_types_in_dictionary(dictionary[k])
#         else:
#             if type(v) is float64:
#                 dictionary[k] = float(v)
#     return dictionary
#
# def test():
#    print msg_value
#    msg_value = replace_types_in_dictionary(msg_value)
# test()
#