from threading import Lock
import rospy
import json

from rosplan_pytools.srv import DiagnosticsDB, ResetDB
from rosplan_pytools.srv import AddElement, FindElement, UpdateElement, RemoveElement, RetrieveElements
from rosplan_pytools.rosparam.common.service_names import ServiceNames
from rosplan_pytools.rosparam.controller.connection import RosParamsConnection

"""
Conversions:
    [client] --- str --> [storage] --- dict --> [ros params]
    [client] <-- str --- [storage] <-- dict --- [ros params]
"""


def _rosparam_to_string(element):
    # type: (dict) -> str
    return json.dumps(element)


def _string_to_rosparam(element):
    # type: (str) -> dict
    return json.loads(element)


class RosParamsStorageServer(object):

    def __init__(self, storage_name='my_storage'):

        self._lock = Lock()
        self._ros_server = RosParamsConnection(storage_name)
        self._start_services(storage_name)

    def _start_services(self, prefix):

        service_prefix = prefix + '/'

        service_name = service_prefix + ServiceNames.DIAGNOSTICS_DB
        rospy.Service(service_name, DiagnosticsDB, self._diagnostics_db)

        service_name = service_prefix + ServiceNames.RESET_DB
        rospy.Service(service_name, ResetDB, self._reset_db)

        service_name = service_prefix + ServiceNames.ADD_ELEMENT
        rospy.Service(service_name, AddElement, self._add_element)

        service_name = service_prefix + ServiceNames.FIND_ELEMENT
        rospy.Service(service_name, FindElement, self._find_element)

        service_name = service_prefix + ServiceNames.UPDATE_ELEMENT
        rospy.Service(service_name, UpdateElement, self._update_element)

        service_name = service_prefix + ServiceNames.REMOVE_ELEMENT
        rospy.Service(service_name, RemoveElement, self._remove_element)

        service_name = service_prefix + ServiceNames.RETRIEVE_ELEMENTS
        rospy.Service(service_name, RetrieveElements, self._retrieve_elements)

    def _diagnostics_db(self, request):

        rospy.loginfo("[RPpt][RpS] _diagnostics_db")

        with self._lock:
            num_elements = self._ros_server.num_elements()

        return True, num_elements

    def _reset_db(self, request):

        rospy.loginfo("[RPpt][RpS] _reset_db")

        with self._lock:
            self._ros_server.reset()
        return True

    def _add_element(self, request):

        rospy.loginfo("[RPpt][RpS] _add_element (%s) = %s", request.key, request.value)

        success = False
        name = request.key
        element = _string_to_rosparam(request.value)

        with self._lock:
            success = self._ros_server.add_element(name, element)

        return success

    def _find_element(self, request):

        rospy.loginfo("[RPpt][RpS] _find_element (%s)", request.key)

        success = False
        name = request.key
        NO_METADATA = ""
        value = ""

        with self._lock:
            element = self._ros_server.get_element(name)
            if len(element.keys()) > 0:
                value = _rosparam_to_string(element)

        return success, NO_METADATA, value

    def _update_element(self, request):

        rospy.loginfo("[RPpt][RpS] _update_element (%s) = %s", request.key, request.value)

        success = False
        name = request.key
        element = _string_to_rosparam(request.value)

        with self._lock:
            success = self._ros_server.update_element(name, element)

        return success

    def _remove_element(self, request):

        rospy.loginfo("[RPpt][RpS] _remove_element (%s)", request.key)

        success = False
        name = request.key

        with self._lock:
            success = self._ros_server.remove_element(name)

        return success

    def _retrieve_elements(self, request):

        rospy.loginfo("[RPpt][RpS] _retrieve_elements")

        keys = []

        with self._lock:
            elements = self._ros_server.get_all_elements()
            for e in elements:
                keys.append(_rosparam_to_string(e))

        return True, keys


DEFAULT_DB_NAME = "my_storage"


def start_node(arguments):

    try:
        rospy.init_node(arguments)

        RosParamsStorageServer(DEFAULT_DB_NAME)

        rospy.set_param(DEFAULT_DB_NAME + '/is_ready', True)
        rospy.loginfo("[RPpt][RP] RosParams Storage: Ready to receive")
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.set_param(DEFAULT_DB_NAME + '/is_ready', False)
