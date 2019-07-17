
from __future__ import absolute_import
import uuid
import rospy
from rosplan_pytools.common import message_converter


class RosServerConnection(object):

    def __init__(self, prefix='scene_database'):

        self._param_prefix = prefix

        if not rospy.has_param(self._create_key('num_elements')):
            rospy.set_param(self._create_key('num_elements'), 0)

    def reset(self):

        for edx in range(0, self._get_num_elements()):
            try:
                id_ = edx + 1
                key = self._create_key('elements/' + str(id_))
                rospy.delete_param(key)
            except KeyError:
                pass

        self._set_num_elements(0)

    def num_elements(self):

        return self._get_num_elements()

    def add_element(self, name, message, metadata=''):

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
        rospy.set_param(self._create_key('elements/' + str(id_)), element)

        self._set_num_elements(self._get_num_elements() + 1)

        return True

    def update_element(self, name, message, metadata=''):

        key, id_ = self._find_element_by_name(name)
        if key == "":
            return False

        element = rospy.get_param(key, dict())
        if not element or element['msg_type'] != message._type:
            return False

        if metadata == '':
            metadata = element['metadata']

        msg_value = message_converter.convert_ros_message_to_dictionary(message)
        updated_element = {'name': name, 'metadata': metadata, 'uuid': element['uuid'],
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
            last_element_key = self._create_key('elements/' + str(element_id))
            element = rospy.get_param(last_element_key)
            rospy.delete_param(last_element_key)
            rospy.set_param(key, element)

        num_elements -= 1
        self._set_num_elements(num_elements)

        return True

    def element_exists(self, name):

        key, id_ = self._find_element_by_name(name)
        return key != ''

    def get_element(self, name):

        key, id_ = self._find_element_by_name(name)
        if key == '':
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
            key = self._create_key('elements/' + str(id_))

            if rospy.has_param(key):

                name = rospy.get_param(key + '/name')
                metadata = rospy.get_param(key + '/metadata')
                msg_type = rospy.get_param(key + '/msg_type')
                msg_value = rospy.get_param(key + '/msg_value')
                uuid_ = rospy.get_param(key + '/uuid')

                message = message_converter.convert_dictionary_to_ros_message(msg_type, msg_value, 'message')
                elements.append((name, message, metadata, uuid_))

            else:
                raise RuntimeError('Data stored is inconsistent.')

        return elements

    def _create_key(self, name):
        return "/" + self._param_prefix + '/' + name

    def _get_num_elements(self):
        return rospy.get_param(self._create_key('num_elements'), 0)

    def _set_num_elements(self, number):
        if number < 0:
            number = 0
        return rospy.set_param(self._create_key('num_elements'), number)

    def _find_element_by_name(self, name):

        found = False
        element_key = ""
        element_id = -1

        for edx in range(0, self._get_num_elements()):
            element_id = edx + 1
            element_key = self._create_key('elements/' + str(element_id))
            if rospy.get_param(element_key + '/name', '') == name:
                found = True
                break

        if not found:
            element_key = ""
            element_id = -1

        return element_key, element_id
