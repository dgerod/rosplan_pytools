"""
 ROSPlan Scene Database interface
 Avoids the totally convoluted syntax of ROSPlan, and lets you easily put data in
 the scene database at the same time.
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

        num_elements = self._get_num_elements()
        for e in range(0, num_elements):
            try:
                key = self._create_key("elements/" + str(e))
                rospy.delete_param(key)
            except KeyError:
                pass

        self._set_num_elements(0)

    def num_elements(self):

        return self._get_num_elements()

    def add_element(self, name, message):

        e, key = self._find_element_by_name(name)
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

        num_elements = self._get_num_elements()
        element = {'name': name, 'id': str(uuid.uuid4()), 'msg_type': message.__class__._type,
                   'msg_value': msg_value}

        rospy.set_param(self._create_key("elements/" + str(num_elements)), element)

        num_elements += 1
        self._set_num_elements(num_elements)

        return True

    def update_element(self, name, message):

        import pdb
        pdb.set_trace()

        e, key = self._find_element_by_name(name)
        if key == "":
            return False

        element = rospy.get_param(key, dict())
        if not element or element['msg_type'] != message._type:
            return False

        updated_element = {'name': name, 'id': element['id'], 'msg_type': element['msg_type'],
                           'msg_value': message_converter.convert_ros_message_to_dictionary(message)}
        rospy.set_param(key, updated_element)

        return True

    def remove_element(self, name):

        edx, key = self._find_element_by_name(name)

        if key == "":
            return False

        num_elements = self._get_num_elements()
        is_last_element = (edx == num_elements-1)

        if is_last_element:
            rospy.delete_param(key)

        else:
            last_element_key = self._create_key("elements/" + str(num_elements-1))
            element = rospy.get_param(last_element_key)
            rospy.delete_param(last_element_key)
            rospy.set_param(key, element)

        num_elements -= 1
        self._set_num_elements(num_elements)

        return True

    def element_exists(self, name):

        e, key = self._find_element_by_name(name)
        return key != ""

    def get_element(self, name):

        edx, key = self._find_element_by_name(name)
        if key == "":
            return False, ()

        element = rospy.get_param(key, dict())
        if element:
            return True, \
                   (message_converter.convert_dictionary_to_ros_message(element['msg_type'],
                                                                        element['msg_value'],
                                                                        'message'),
                    element['id'])
        else:
            return False, ()

    def get_all_elements(self):

        num_elements = self._get_num_elements()
        elements = list()

        for edx in range(0, num_elements):
            key = self._create_key("elements/" + str(edx))
            if rospy.has_param(key):
                name = rospy.get_param(key + "/name")
                msg_type = rospy.get_param(key + "/msg_type")
                msg_value = rospy.get_param(key + "/msg_value")
                id_ = rospy.get_param(key + "/id")

                message = message_converter.convert_dictionary_to_ros_message(msg_type, msg_value, 'message')
                elements.append((name, message, id_))

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
        edx = -1
        key = ""

        num_elements = self._get_num_elements()
        for edx in range(0, num_elements):
            key = self._create_key("elements/" + str(edx))
            if rospy.get_param(key + "/name", "") == name:
                found = True
                break

        if not found:
            edx = -1
            key = ""

        return edx, key


class Element(object):

    @staticmethod
    def extract_ros_type(message):
        return message._type

    def __init__(self, value=None):

        if value is not None:
            self._value = value
            self._ros_type = self.extract_ros_type(value)

    def __str__(self):

        if self._value is not None:
            value = self._value
        else:
            value = "None"

        return "%s, %s" % (self._ros_type, value)

    def __eq__(self, other):

        return self._value == other._value

    def clean(self):
        self._value = None
        self._ros_type = ""

    def is_valid(self):
        return self._value is not None

    def value(self):
        return self._value

    def ros_type(self):
        return self._ros_type


def _find_element_by_name(name):

    if not _ros_server.element_exists(name):
        return Element()

    success, element = _ros_server.get_element(name)
    if not success:
        return Element()
    else:
        return Element(element[0])


def init(sdb_name=None):

    global _ros_server

    if sdb_name is None:
        sdb_name = "scene_database"

    _ros_server = _RosServerConnection(sdb_name)


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
    for name, message, id_ in elements:
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

        _ros_server.add_element(name, element.value())

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


def clear_elements():

    elements = _ros_server.get_all_elements()
    for name, message, id_ in elements:
        _ros_server.remove_element(name)


def reset():

    _ros_server.reset()
