"""
 ROSPlan Scene Database interface
 Lets you easily put data in the scene database that it is implemented
 using Ros Server
"""

from __future__ import absolute_import
import uuid
import rospy
from rosplan_pytools.common import message_converter


_ros_server = None


class _RosServerConnection(object):

    def __init__(self, prefix="scene_database"):

        self._param_prefix = prefix

        if not rospy.has_param(self._create_key("num_elements")):
            rospy.set_param(self._create_key("num_elements"), 0)

    def reset(self):

        for edx in range(0, self._get_num_elements()):
            try:
                id_ = edx + 1
                key = self._create_key("elements/" + str(id_))
                rospy.delete_param(key)
            except KeyError:
                pass

        self._set_num_elements(0)

    def num_elements(self):

        return self._get_num_elements()

    def add_element(self, name, message, metadata=""):

        key, id_ = self._find_element_by_name(name)
        if key != "":
            return False

        msg_value = message_converter.convert_ros_message_to_dictionary(message)

        # def replace_types_in_dictionary(dictionary):
        #
        #     for k, v in dictionary.iteritems():
        #         if type(v) is dict:
        #             replace_types_in_dictionary(dictionary[k])
        #         else:
        #             if type(v) is float64:
        #                 dictionary[k] = float(v)
        #
        #     return dictionary
        #
        # print msg_value
        # msg_value = replace_types_in_dictionary(msg_value)

        element = {'name': name, 'metadata': metadata, 'uuid': str(uuid.uuid4()),
                   'msg_type': message.__class__._type,'msg_value': msg_value}

        id_ = self._get_num_elements() + 1
        rospy.set_param(self._create_key("elements/" + str(id_)), element)

        self._set_num_elements(self._get_num_elements() + 1)

        return True

    def update_element(self, name, message):

        key, id_ = self._find_element_by_name(name)
        if key == "":
            return False

        element = rospy.get_param(key, dict())
        if not element or element['msg_type'] != message._type:
            return False

        msg_value = message_converter.convert_ros_message_to_dictionary(message)
        updated_element = {'name': name, 'metadata': element['metadata'], 'uuid': element['uuid'],
                           'msg_type': element['msg_type'], 'msg_value': msg_value}
        rospy.set_param(key, updated_element)

        return True

    def remove_element(self, name):

        key, id_ = self._find_element_by_name(name)

        if key == "":
            return False

        num_elements = self._get_num_elements()
        is_last_element = (id_ == self._get_num_elements())

        if is_last_element:
            rospy.delete_param(key)

        else:
            element_id = num_elements
            last_element_key = self._create_key("elements/" + str(element_id))
            element = rospy.get_param(last_element_key)
            rospy.delete_param(last_element_key)
            rospy.set_param(key, element)

        num_elements -= 1
        self._set_num_elements(num_elements)

        return True

    def element_exists(self, name):

        key, id_ = self._find_element_by_name(name)
        return key != ""

    def get_element(self, name):

        key, id_ = self._find_element_by_name(name)
        if key == "":
            return False, ()

        element = rospy.get_param(key, dict())
        if element:
            return True, \
                   (message_converter.convert_dictionary_to_ros_message(element['msg_type'],
                                                                        element['msg_value'],
                                                                        'message'),
                    element['metadata'],
                    element['uuid'])
        else:
            return False, ()

    def get_all_elements(self):

        elements = list()
        for edx in range(0, self._get_num_elements()):

            id_ = edx + 1
            key = self._create_key("elements/" + str(id_))

            if rospy.has_param(key):

                name = rospy.get_param(key + "/name")
                metadata = rospy.get_param(key + "/metadata")
                msg_type = rospy.get_param(key + "/msg_type")
                msg_value = rospy.get_param(key + "/msg_value")
                uuid_ = rospy.get_param(key + "/uuid")

                message = message_converter.convert_dictionary_to_ros_message(msg_type, msg_value, 'message')
                elements.append((name, message, metadata, uuid_))

            else:
                raise RuntimeError("Data stored is inconsistent.")

        return elements

    def _create_key(self, name):
        return "/" + self._param_prefix + "/" + name

    def _get_num_elements(self):
        return rospy.get_param(self._create_key("num_elements"), 0)

    def _set_num_elements(self, number):
        if number < 0:
            number = 0
        return rospy.set_param(self._create_key("num_elements"), number)

    def _find_element_by_name(self, name):

        found = False
        element_key = ""
        element_id = -1

        for edx in range(0, self._get_num_elements()):
            element_id = edx + 1
            element_key = self._create_key("elements/" + str(element_id))
            if rospy.get_param(element_key + "/name", "") == name:
                found = True
                break

        if not found:
            element_key = ""
            element_id = -1

        return element_key, element_id


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


def _find_element_by_name(name):

    if not _ros_server.element_exists(name):
        return Element()

    success, element = _ros_server.get_element(name)
    if not success:
        return Element()
    else:
        return Element(element[0], element[1])


def initialize(sdb_name=None):

    global _ros_server

    if sdb_name is None:
        sdb_name = "scene_database"

    _ros_server = _RosServerConnection(sdb_name)


def num_elements():

    return _ros_server.num_elements()


def exist_element(name):
    """
    :param name: name of the element (str)
    :return: success or failure
    """

    if not isinstance(name, str):
        raise TypeError

    return _ros_server.element_exists(name)


def list_elements():

    names = list()
    elements = _ros_server.get_all_elements()
    for name, message, metadata, id_ in elements:
        names.append(name)

    return names


def add_element(name, element):
    """
    :param name: name of the element (str)
    :param element: an element instance (Element)
    :return: success or failure
    """

    if not isinstance(name, str) or not isinstance(element, Element):
        raise TypeError

    success = False

    if not exist_element(name) and element.is_valid():

        _ros_server.add_element(name, element.value(), element.metadata())

        updated_element = _find_element_by_name(name)
        success = updated_element.is_valid()

    return success


def update_element(name, element):
    """
    :param name: name of the element (str)
    :param element: an element instance (Element)
    :return: success or failure
    """

    if not isinstance(name, str) or not isinstance(element, Element):
        raise TypeError

    success = False
    registered_element = _find_element_by_name(name)

    if registered_element.is_valid() and registered_element == element:

        _ros_server.update_element(name, element.value())

        updated_element = _find_element_by_name(name)
        success = updated_element.is_valid()

    return success


def get_element(name):
    """
    :param name:
    :return: success or failure
    """

    if not isinstance(name, str):
        raise TypeError

    element = _find_element_by_name(name)

    return element.is_valid(), element


def remove_element(name):
    """
    :param name:
    :return: success or failure
    """

    if not isinstance(name, str):
        raise TypeError

    if _find_element_by_name(name).is_valid():

        success = _ros_server.remove_element(name)
        return success

    else:
        return False


def remove_all_elements():

    elements = _ros_server.get_all_elements()
    for name, message, id_ in elements:
        _ros_server.remove_element(name)


def reset():

    _ros_server.reset()
