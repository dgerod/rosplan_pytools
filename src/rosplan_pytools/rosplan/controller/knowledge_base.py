"""
 ROSPlan Knowledge Base interface
 Avoids the totally convoluted syntax of ROSPlan.
"""

from __future__ import absolute_import

import rospy
from std_srvs.srv import Empty
from rosplan_knowledge_msgs.srv import \
    KnowledgeUpdateService, KnowledgeQueryService, \
    GetInstanceService, GetAttributeService, GetDomainAttributeService, \
    GetDomainTypeService, GetDomainOperatorService, \
    GetDomainOperatorDetailsService, GetDomainPredicateDetailsService
from rosplan_knowledge_msgs.msg import KnowledgeItem
from rosplan_pytools.rosplan.common.utils import keyval_to_dict, dict_to_keyval


KB_UPDATE_ADD_KNOWLEDGE = 0
KB_UPDATE_RM_KNOWLEDGE = 2
KB_UPDATE_ADD_GOAL = 1
KB_UPDATE_RM_GOAL = 3

KB_ITEM_INSTANCE = 0
KB_ITEM_FACT = 1
KB_ITEM_FUNCTION = 2

_services = {}


def _initialize_kb_services(prefix):

    global _services

    _services["get_current_instances"] = \
        rospy.ServiceProxy(prefix + "/get_current_instances",
                           GetInstanceService)
    _services["update_knowledge_base"] = \
        rospy.ServiceProxy(prefix + "/update_knowledge_base",
                           KnowledgeUpdateService)
    _services["get_domain_predicates"] = \
        rospy.ServiceProxy(prefix + "/get_domain_predicates",
                           GetDomainAttributeService)
    _services["get_domain_predicate_details"] = \
        rospy.ServiceProxy(prefix + "/get_domain_predicate_details",
                           GetDomainPredicateDetailsService)
    _services["get_current_goals"] = \
        rospy.ServiceProxy(prefix + "/get_current_goals",
                           GetAttributeService)
    _services["get_current_knowledge"] = \
        rospy.ServiceProxy(prefix + "/get_current_knowledge",
                           GetAttributeService)
    _services["get_domain_operators"] = \
        rospy.ServiceProxy(prefix + "/get_domain_operators",
                           GetDomainOperatorService)
    _services["get_domain_operator_details"] = \
        rospy.ServiceProxy(prefix + "/get_domain_operator_details",
                           GetDomainOperatorDetailsService)
    _services["get_domain_types"] = \
        rospy.ServiceProxy(prefix + "/get_domain_types",
                           GetDomainTypeService)
    _services["planning"] = \
        rospy.ServiceProxy(prefix + "/planning_server",
                           Empty)
    _services["query"] = \
        rospy.ServiceProxy(prefix + "/query_knowledge_base",
                           KnowledgeQueryService)
    _services["clear_knowledge"] = \
        rospy.ServiceProxy(prefix + "/clear_knowledge_base",
                           Empty)


def _is_predicate_negative(name):
    """
    Checks and remove negative symbol in case it the predicate is negated.
    The accepted format is: "[not|NOT|!] predicate_name", it is mandatory a space 
    between the negative symbol and the predicate  name.
    """

    pattern_1 = "not "
    pattern_2 = pattern_1.upper()
    pattern_3 = "! "

    # Check if the name is negated, and in case it is negated remove
    # negative symbol
    if name.startswith(pattern_1) or name.startswith(pattern_2):
        index = len(pattern_1)
        new_name = name[index:]
        is_negative = True
    elif name.startswith(pattern_3):
        index = len(pattern_3)
        new_name = name[index:]
        is_negative = True
    else:
        new_name = name
        is_negative = False

    return new_name, is_negative


def _generate_predicate(type_name, **kwargs):
    new_type_name, is_negative = _is_predicate_negative(type_name)
    return KnowledgeItem(KB_ITEM_FACT,
                         "",
                         "",
                         new_type_name,
                         dict_to_keyval(kwargs),
                         0.0, is_negative)


def _instance_exists(item_name):
    instance_names = _services["get_current_instances"]("").instances
    return item_name in instance_names


def initialize(prefix=None):

    if prefix is None:
        prefix = "/kcl_rosplan"

    _initialize_kb_services(prefix)


def exist_instance(item):
    return _instance_exists(item)


def add_instance(item_name, type_name):

    if not isinstance(item_name, str) or not isinstance(type_name, str):
        raise TypeError

    return _services["update_knowledge_base"](KB_UPDATE_ADD_KNOWLEDGE,
                                              KnowledgeItem(KB_ITEM_INSTANCE,
                                                            type_name,
                                                            item_name,
                                                            "", [], 0.0, False)).success


def remove_instance(item_name, type_name):

    if not isinstance(item_name, str) or not isinstance(type_name, str):
        raise TypeError

    return _services["update_knowledge_base"](KB_UPDATE_RM_KNOWLEDGE,
                                              KnowledgeItem(KB_ITEM_INSTANCE,
                                                            type_name,
                                                            item_name,
                                                            "", [], 0.0, False)).success


def get_instance_type(item_name):

    if not isinstance(item_name, str):
        raise TypeError

    types = _services["get_domain_types"]().types
    for type_name in types:
        instance_names = _services["get_current_instances"](type_name).instances
        if item_name in instance_names:
            return type_name

    return ""


def list_instances(type_name=""):
    instance_names = _services["get_current_instances"](type_name).instances
    return instance_names


def remove_all_instances():

    # todo-q dgerod@ could a instance be deleted only knowing its name???
    raise NotImplementedError

    for item_name in list_instances():
        #instance, type_name = get_instance(None, item_name)
        remove_instance(type_name, item_name)


def add_predicate(type_name, **kwargs):
    if isinstance(type_name, KnowledgeItem):
        return _services["update_knowledge_base"](KB_UPDATE_ADD_KNOWLEDGE, type_name).success
    return _services["update_knowledge_base"](KB_UPDATE_ADD_KNOWLEDGE,
                                              _generate_predicate(type_name, **kwargs)).success


def remove_predicate(type_name, **kwargs):
    if isinstance(type_name, KnowledgeItem):
        return _services["update_knowledge_base"](KB_UPDATE_RM_KNOWLEDGE, type_name).success
    return _services["update_knowledge_base"](KB_UPDATE_RM_KNOWLEDGE,
                                              _generate_predicate(type_name, **kwargs)).success


def list_predicates():
    predicates = _services["get_current_knowledge"]("").attributes
    return predicates


def remove_all_predicates():
    for predicate in list_predicates():
        remove_predicate(predicate)


def add_goal(type_name, **kwargs):
    if isinstance(type_name, KnowledgeItem):
        return _services["update_knowledge_base"](KB_UPDATE_ADD_GOAL, type_name)
    return _services["update_knowledge_base"](KB_UPDATE_ADD_GOAL,
                                              _generate_predicate(type_name, **kwargs)).success


def remove_goal(type_name, **kwargs):
    if isinstance(type_name, KnowledgeItem):
        return _services["update_knowledge_base"](KB_UPDATE_RM_GOAL, type_name).success.success
    return _services["update_knowledge_base"](KB_UPDATE_RM_GOAL,
                                              _generate_predicate(type_name, **kwargs)).success


def list_goals():
    goals = _services["get_current_goals"]("").attributes
    return goals


def remove_all_goals():
    for goal in list_goals():
        remove_goal(goal)


def get_args(item):
    domain_items = dict()
    res = _services["get_domain_predicates"]()
    for predicate in res.items:
        domain_items[predicate.name] = keyval_to_dict(predicate.typed_parameters)
    if item not in domain_items:
        return False  # not in domain...
    else:
        return domain_items[item].keys()


def reset():
    _services["clear_knowledge"]()
