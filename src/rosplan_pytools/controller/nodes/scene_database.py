from threading import Lock
import rospy
from rosplan_pytools.controller.common.sdb_element import Element
from rosplan_pytools.controller.common.sdb_element_converter import sdb_element_to_string, string_to_sdb_element
from rosplan_pytools.controller.nodes.ros_server_connection import RosServerConnection
from rosplan_pytools.srv import DiagnosticsDB, ResetDB
from rosplan_pytools.srv import AddElement, FindElement, UpdateElement, RemoveElement, RetrieveElements


class ServiceNames(object):

    DIAGNOSTICS_DB = 'diagnostics_db'
    RESET_DB = 'reset_db'

    ADD_ELEMENT = 'add_element'
    REMOVE_ELEMENT = 'remove_element'
    FIND_ELEMENT = 'find_element'
    UPDATE_ELEMENT = 'update_element'
    RETRIEVE_ELEMENTS = 'retrieve_elements'


class SceneDatabase(object):

    def __init__(self, service_prefix, sdb_name='scene_database'):

        self._lock = Lock()
        self._ros_server = RosServerConnection(sdb_name)
        self.start_services(service_prefix)

    def start_services(self, prefix):

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

        rospy.loginfo("[RPpt][SDB] _diagnostics_db")

        with self._lock:
            num_elements = self._ros_server.num_elements()

        return True, num_elements

    def _reset_db(self, request):

        rospy.loginfo("[RPpt][SDB] _reset_db")

        with self._lock:
            self._ros_server.reset()
        return True

    def _add_element(self, request):

        rospy.loginfo("[RPpt][SDB] _add_element")

        success = False
        element = string_to_sdb_element(request.metadata, request.value)
        rospy.loginfo("[RPpt][SDB] element: %s = %s", request.key, request.value)

        with self._lock:
            if not self._ros_server.element_exists(request.key) and request.value != '':
                success = self._ros_server.add_element(request.key, element.value(), element.metadata())

        return success

    def _find_element(self, request):

        rospy.loginfo("[RPpt][SDB] _find_element")

        success = False
        metadata = ""
        value = ""

        with self._lock:
            if self._ros_server.element_exists(request.key):
                success, result = self._ros_server.get_element(request.key)
                if success:
                    metadata, value = sdb_element_to_string(Element(result[0], result[1]))

        return success, metadata, value

    def _update_element(self, request):

        rospy.loginfo("[RPpt][SDB] _update_element")
        rospy.loginfo("[RPpt][SDB] element %s = %s", request.key, request.value)

        success = False
        element = string_to_sdb_element(request.metadata, request.value)

        with self._lock:
            if self._ros_server.element_exists(request.key) and request.value != "":
                success = self._ros_server.update_element(request.key, element.value(), element.metadata())

        return success

    def _remove_element(self, request):

        rospy.loginfo("[RPpt][SDB] _remove_element")

        success = False
        key = request.key

        with self._lock:
            if self._ros_server.element_exists(key):
                success = self._ros_server.remove_element(key)

        return success

    def _retrieve_elements(self, request):

        rospy.loginfo("retrieve_elements")

        keys = []

        with self._lock:
            elements = self._ros_server.get_all_elements()
            for e in elements:
                keys.append(e[0])

        return True, keys

    def _remove_all_elements(self):

        rospy.loginfo("remove_all_elements")

        with self._lock:
            elements = self._ros_server.get_all_elements()
            for key, value, metadata, id_ in elements:
                self._ros_server.remove_element(key)

        return True


def start_node(node_name, database_name):

    try:
        rospy.init_node(node_name)
        SceneDatabase(node_name, database_name)

        rospy.set_param(node_name + '/is_ready', True)
        rospy.loginfo("Scene Database: Ready to receive")
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.set_param(node_name + '/is_ready', False)
