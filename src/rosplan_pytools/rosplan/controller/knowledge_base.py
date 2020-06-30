"""
 ROSPlan Knowledge Base interface
 Avoids the totally convoluted syntax of ROSPlan.
"""

from __future__ import absolute_import
from collections import OrderedDict

import rospy
from std_srvs.srv import Empty
from rosplan_knowledge_msgs.srv import \
    KnowledgeUpdateService, KnowledgeQueryService, \
    GetInstanceService, GetAttributeService, GetDomainAttributeService, \
    GetDomainTypeService, GetDomainOperatorService, \
    GetDomainOperatorDetailsService, GetDomainPredicateDetailsService
from rosplan_knowledge_msgs.msg import KnowledgeItem
from rosplan_pytools.rosplan.common.utils import keyval_to_dict, dict_to_keyval


DEFAULT_SERVICE_PREFIX = "/rosplan_knowledge_base"

KB_UPDATE_ADD_KNOWLEDGE = 0 # KnowledgeUpdateService().ADD_KNOWLEDGE
KB_UPDATE_ADD_GOAL = 1 # KnowledgeUpdateService().ADD_GOAL
KB_UPDATE_RM_KNOWLEDGE = 2 # KnowledgeUpdateService().REMOVE_KNOWLEDGE
KB_UPDATE_RM_GOAL = 3 # KnowledgeUpdateService().REMOVE_GOAL

KB_ITEM_INSTANCE = KnowledgeItem().INSTANCE
KB_ITEM_FACT = KnowledgeItem().FACT
KB_ITEM_FUNCTION = KnowledgeItem().FUNCTION


_services = {}


def _wait_until_kb_is_ready(service_naem):
    rospy.wait_for_service(service_naem)


def _initialize_kb_services(prefix):

    global _services

    _services["get_domain_predicates"] = \
        rospy.ServiceProxy(prefix + "/domain/predicates",
                           GetDomainAttributeService)
    _services["get_domain_predicate_details"] = \
        rospy.ServiceProxy(prefix + "/domain/predicate_details",
                           GetDomainPredicateDetailsService)
    _services["get_domain_operators"] = \
        rospy.ServiceProxy(prefix + "/domain/operators",
                           GetDomainOperatorService)
    _services["get_domain_operator_details"] = \
        rospy.ServiceProxy(prefix + "/domain/operators_details",
                           GetDomainOperatorDetailsService)
    _services["get_domain_types"] = \
        rospy.ServiceProxy(prefix + "/domain/types",
                           GetDomainTypeService)

    _services["get_current_instances"] = \
        rospy.ServiceProxy(prefix + "/state/instances",
                           GetInstanceService)
    _services["get_current_goals"] = \
        rospy.ServiceProxy(prefix + "/state/goals",
                           GetAttributeService)
    _services["get_propositions"] = \
        rospy.ServiceProxy(prefix + "/state/propositions",
                           GetAttributeService)

    _services["query_knowledge_base"] = \
        rospy.ServiceProxy(prefix + "/query_state",
                           KnowledgeQueryService)
    _services["update_knowledge_base"] = \
        rospy.ServiceProxy(prefix + "/update",
                           KnowledgeUpdateService)
    _services["clear_knowledge"] = \
        rospy.ServiceProxy(prefix + "/clear",
                           Empty)

    _wait_until_kb_is_ready(prefix + "/clear")


def _is_predicate_negative(name):
    """
    Checks and remove negative symbol in case the predicate is negated.
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


def _make_instance(type_name, item_name):

    kb_item = KnowledgeItem()
    kb_item.knowledge_type = KB_ITEM_INSTANCE
    kb_item.instance_name = item_name
    kb_item.instance_type = type_name
    return kb_item


def _make_predicate(type_name, **kwargs):

    new_type_name, is_negative = _is_predicate_negative(type_name)
    kb_item = KnowledgeItem()
    kb_item.knowledge_type = KB_ITEM_FACT
    kb_item.is_negative = is_negative
    kb_item.attribute_name = new_type_name
    kb_item.values = dict_to_keyval(kwargs)
    return kb_item


def _instance_exists(item_name):
    instance_names = _services["get_current_instances"]("").instances
    return item_name in instance_names


def initialize(prefix=None):
    service_prefix = prefix or DEFAULT_SERVICE_PREFIX
    _initialize_kb_services(service_prefix)


def exist_instance(item):
    return _instance_exists(item)


def add_instance(item_name, type_name):

    if not isinstance(item_name, str) or not isinstance(type_name, str):
        raise TypeError

    return _services["update_knowledge_base"](KB_UPDATE_ADD_KNOWLEDGE,
                                              _make_instance(type_name, item_name)).success


def remove_instance(item_name, type_name):

    if not isinstance(item_name, str) or not isinstance(type_name, str):
        raise TypeError

    return _services["update_knowledge_base"](KB_UPDATE_RM_KNOWLEDGE,
                                              _make_instance(type_name, item_name)).success


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
    for item_name in list_instances():
        type_name = get_instance_type(item_name)
        remove_instance(item_name, type_name)


def add_predicate(type_name, **kwargs):

    if isinstance(type_name, KnowledgeItem):
        return _services["update_knowledge_base"](KB_UPDATE_ADD_KNOWLEDGE, type_name).success
    else:
        return _services["update_knowledge_base"](KB_UPDATE_ADD_KNOWLEDGE,
                                                  _make_predicate(type_name, **kwargs)).success

def remove_predicate(type_name, **kwargs):
    if isinstance(type_name, KnowledgeItem):
        return _services["update_knowledge_base"](KB_UPDATE_RM_KNOWLEDGE, type_name).success
    else:
        return _services["update_knowledge_base"](KB_UPDATE_RM_KNOWLEDGE,
                                                  _make_predicate(type_name, **kwargs)).success


def list_predicates():
    predicates = _services["get_propositions"]().attributes
    return predicates


def remove_all_predicates():
    for predicate in list_predicates():
        remove_predicate(predicate)


def add_goal(type_name, **kwargs):
    if isinstance(type_name, KnowledgeItem):
        return _services["update_knowledge_base"](KB_UPDATE_ADD_GOAL, type_name).success
    else:
        return _services["update_knowledge_base"](KB_UPDATE_ADD_GOAL,
                                                  _make_predicate(type_name, **kwargs)).success


def remove_goal(type_name, **kwargs):
    if isinstance(type_name, KnowledgeItem):
        return _services["update_knowledge_base"](KB_UPDATE_RM_GOAL, type_name).success
    else:
        return _services["update_knowledge_base"](KB_UPDATE_RM_GOAL,
                                                  _make_predicate(type_name, **kwargs)).success


def list_goals():
    goals = _services["get_current_goals"]("").attributes
    return goals


def remove_all_goals():
    for goal in list_goals():
        remove_goal(goal)


def reset():
    _services["clear_knowledge"]()


def remove_all():
    _services["clear_knowledge"]()
    remove_all_predicates()
