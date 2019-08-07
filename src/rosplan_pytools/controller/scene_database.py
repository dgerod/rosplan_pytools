import rospy
from rosplan_pytools.controller.common.sdb_element import Element
from rosplan_pytools.controller.common.sdb_element_converter import sdb_element_to_string, string_to_sdb_element
from rosplan_pytools.controller.nodes.scene_database import ServiceNames
from rosplan_pytools.srv import DiagnosticsDB, ResetDB
from rosplan_pytools.srv import AddElement, FindElement, UpdateElement, RemoveElement, RetrieveElements


_services = {}


def _initialize_services(sdb_name):

    global _services

    service_name = sdb_name + "/" + ServiceNames.DIAGNOSTICS_DB
    rospy.wait_for_service(service_name)
    _services[ServiceNames.DIAGNOSTICS_DB] = rospy.ServiceProxy(service_name, DiagnosticsDB)

    service_name = sdb_name + "/" + ServiceNames.RESET_DB
    rospy.wait_for_service(service_name)
    _services[ServiceNames.RESET_DB] = rospy.ServiceProxy(service_name, ResetDB)

    service_name = sdb_name + "/" + ServiceNames.ADD_ELEMENT
    rospy.wait_for_service(service_name)
    _services[ServiceNames.ADD_ELEMENT] = rospy.ServiceProxy(service_name, AddElement)

    service_name = sdb_name + "/" + ServiceNames.FIND_ELEMENT
    rospy.wait_for_service(service_name)
    _services[ServiceNames.FIND_ELEMENT] = rospy.ServiceProxy(service_name, FindElement)

    service_name = sdb_name + "/" + ServiceNames.UPDATE_ELEMENT
    rospy.wait_for_service(service_name)
    _services[ServiceNames.UPDATE_ELEMENT] = rospy.ServiceProxy(service_name, UpdateElement)

    service_name = sdb_name + "/" + ServiceNames.REMOVE_ELEMENT
    rospy.wait_for_service(service_name)
    _services[ServiceNames.REMOVE_ELEMENT] = rospy.ServiceProxy(service_name, RemoveElement)

    service_name = sdb_name + "/" + ServiceNames.RETRIEVE_ELEMENTS
    rospy.wait_for_service(service_name)
    _services[ServiceNames.RETRIEVE_ELEMENTS] = rospy.ServiceProxy(service_name, RetrieveElements)


def initialize(sdb_name="scene_database"):

    _initialize_services(sdb_name)


def reset():

    return _services[ServiceNames.RESET_DB]().success


def num_elements():

    return _services[ServiceNames.DIAGNOSTICS_DB]().num_elements


def exist_element(name):

    return _services[ServiceNames.FIND_ELEMENT](name).success


def list_elements():

    return _services[ServiceNames.RETRIEVE_ELEMENTS]().keys


def add_element(name, element):

    if not isinstance(name, str) or not isinstance(element, Element):
        raise TypeError

    metadata, value = sdb_element_to_string(element)
    return _services[ServiceNames.ADD_ELEMENT](name, metadata, value).success


def update_element(name, element):

    if not isinstance(name, str) or not isinstance(element, Element):
        raise TypeError

    metadata, value = sdb_element_to_string(element)
    return _services[ServiceNames.UPDATE_ELEMENT](name, element.metadata(), value).success


def get_element(name):

    if not isinstance(name, str):
        raise TypeError

    element = Element()

    response = _services[ServiceNames.FIND_ELEMENT](name)
    if response.success:
        element = string_to_sdb_element(response.metadata, response.value)

    return element.is_valid(), element


def remove_element(name):

    _services[ServiceNames.REMOVE_ELEMENT](name)


def remove_elements():

    keys = _services[ServiceNames.RETRIEVE_ELEMENTS]().keys
    for k in keys:
        _services[ServiceNames.REMOVE_ELEMENT](k)
