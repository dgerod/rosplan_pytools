
from __future__ import absolute_import
from typing import List
import uuid
import rospy


class RosParamsServerConnection(object):

    def __init__(self, prefix):

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

    def add_element(self, name, element):
        # type: (str, dict) -> bool

        key, id_ = self._find_element_by_name(name)
        if key != '':
            return False

        data = element
        data['name'] = name
        data['uuid'] = str(uuid.uuid4())

        id_ = self._get_num_elements() + 1
        rospy.set_param(self._create_key('elements/' + str(id_)), data)

        self._set_num_elements(self._get_num_elements() + 1)

        return True

    def update_element(self, name, element):
        # type: (str, dict) -> bool

        success = False

        key, id_ = self._find_element_by_name(name)
        if key != '':

            data = rospy.get_param(key, dict())

            updated_element = element
            updated_element['name'] = data['name']
            updated_element['uuid'] = data['uuid']

            rospy.set_param(key, updated_element)
            success = True

        return success

    def remove_element(self, name):
        # type: (str) -> bool

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
        # type: (str) -> bool

        key, id_ = self._find_element_by_name(name)
        return key != ''

    def get_element(self, name):
        # type: (str) -> dict

        element = {}

        key, id_ = self._find_element_by_name(name)

        if key != '':
            data = rospy.get_param(key, dict())

            element = data
            del element['name']
            del element['uuid']

        return element

    def get_all_elements(self):
        # type: () -> List[dict]

        elements = list()

        for edx in range(0, self._get_num_elements()):

            id_ = edx + 1
            key = self._create_key('elements/' + str(id_))

            if rospy.has_param(key):

                data = rospy.get_param(key)

                element = data
                del element['name']
                del element['uuid']

                elements.append(element)

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
