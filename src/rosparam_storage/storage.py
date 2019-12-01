import rospy

from rosplan_pytools.srv import DiagnosticsDB, ResetDB
from rosplan_pytools.srv import AddElement, FindElement, UpdateElement, RemoveElement, RetrieveElements

from rosparam_storage.common.element import StorageElement
from rosparam_storage.common.service_names import ServiceNames

_services = {}


def _initialize_services(storage_name):
    # type: (str) -> None

    global _services

    name = storage_name + "/" + ServiceNames.DIAGNOSTICS_DB
    rospy.wait_for_service(name)
    _services[ServiceNames.DIAGNOSTICS_DB] = rospy.ServiceProxy(name, DiagnosticsDB)

    name = storage_name + "/" + ServiceNames.RESET_DB
    rospy.wait_for_service(name)
    _services[ServiceNames.name] = rospy.ServiceProxy(name, ResetDB)

    name = storage_name + "/" + ServiceNames.ADD_ELEMENT
    rospy.wait_for_service(name)
    _services[ServiceNames.ADD_ELEMENT] = rospy.ServiceProxy(name, AddElement)

    name = storage_name + "/" + ServiceNames.FIND_ELEMENT
    rospy.wait_for_service(name)
    _services[ServiceNames.FIND_ELEMENT] = rospy.ServiceProxy(name, FindElement)

    name = storage_name + "/" + ServiceNames.UPDATE_ELEMENT
    rospy.wait_for_service(name)
    _services[ServiceNames.UPDATE_ELEMENT] = rospy.ServiceProxy(name, UpdateElement)

    name = storage_name + "/" + ServiceNames.REMOVE_ELEMENT
    rospy.wait_for_service(name)
    _services[ServiceNames.REMOVE_ELEMENT] = rospy.ServiceProxy(name, RemoveElement)

    name = storage_name + "/" + ServiceNames.RETRIEVE_ELEMENTS
    rospy.wait_for_service(name)
    _services[ServiceNames.RETRIEVE_ELEMENTS] = rospy.ServiceProxy(name, RetrieveElements)


def initialize(storage_name="my_storage"):
    # type: (str) -> None

    _initialize_services(storage_name)


def reset():
    # type: () -> bool

    return _services[ServiceNames.RESET_DB]().success


def num_elements():
    # type: () -> int

    return _services[ServiceNames.DIAGNOSTICS_DB]().num_elements


def exist_element(key):
    # type: (str) -> bool

    return _services[ServiceNames.FIND_ELEMENT](key).success


def list_elements():
    # type: () -> List[str]

    elements = list()

    keys = _services[ServiceNames.RETRIEVE_ELEMENTS]().keys
    for k in keys:
        elements.append(StorageElement.from_string(k))

    return elements


def add_element(key, element):
    # type: (str, StorageElement) -> bool

    if not isinstance(key, str) or not isinstance(element, StorageElement):
        raise TypeError

    value = StorageElement.to_string(element)
    NO_METADATA = ''

    return _services[ServiceNames.ADD_ELEMENT](key, NO_METADATA, value).success


def update_element(key, element):
    # type: (str, StorageElement) -> bool

    if not isinstance(key, str) or not isinstance(element, StorageElement):
        raise TypeError

    value = StorageElement.to_string(element)
    NO_METADATA = ''

    return _services[ServiceNames.UPDATE_ELEMENT](key, NO_METADATA, value).success


def retrieve_element(key):
    # type: (str) -> StorageElement

    if not isinstance(key, str):
        raise TypeError

    element = StorageElement()

    response = _services[ServiceNames.FIND_ELEMENT](key)
    if response.success:
        element = StorageElement.from_string(response.value)

    return element


def remove_element(key):
    # type: (str) -> None

    _services[ServiceNames.REMOVE_ELEMENT](key)


def remove_elements():

    keys = _services[ServiceNames.RETRIEVE_ELEMENTS]().keys
    for k in keys:
        _services[ServiceNames.REMOVE_ELEMENT](k)
