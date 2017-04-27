"""
 ROSPlan Knowledge Base Interface

 Avoids the totally convoluted syntax of ROSPlan, and lets you easily put data in
 the scene database at the same time.
"""

from __future__ import absolute_import

import rospy
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService, \
    GetDomainAttributeService, GetInstanceService, GetAttributeService, \
    GetDomainOperatorService, GetDomainTypeService, KnowledgeQueryService
from std_srvs.srv import Empty
from rosplan_knowledge_msgs.msg import KnowledgeItem
from .utils import keyval_to_dict, dict_to_keyval
from mongodb_store.message_store import MessageStoreProxy

KB_UPDATE_ADD_KNOWLEDGE = 0
KB_UPDATE_RM_KNOWLEDGE = 2
KB_UPDATE_ADD_GOAL = 1
KB_UPDATE_RM_GOAL = 3

KB_ITEM_INSTANCE = 0
KB_ITEM_FACT = 1
KB_ITEM_FUNCTION = 2

# globals must be explicit
domainitems = {}
services = {}
update_knowledge_base_srv = None
get_domain_predicates_srv = None
get_current_instances_srv = None
db = None

types = {}

def init_kb(prefix=None):
    if prefix is None:
        prefix = "/kcl_rosplan"

    global db
    global services

    services['get_current_instances'] = \
        rospy.ServiceProxy(prefix + "/get_current_instances",
                           GetInstanceService)
    services['update_knowledge_base'] = \
        rospy.ServiceProxy(prefix + "/update_knowledge_base",
                           KnowledgeUpdateService)
    services['get_domain_predicates'] = \
        rospy.ServiceProxy(prefix + "/get_domain_predicates",
                           GetDomainAttributeService)
    services['get_current_goals'] =\
        rospy.ServiceProxy(prefix + "/get_current_goals",
                           GetAttributeService)
    services['get_current_knowledge'] = \
        rospy.ServiceProxy(prefix + "/get_current_knowledge",
                           GetAttributeService)
    services['get_domain_operators'] = \
        rospy.ServiceProxy(prefix + "/get_domain_operators",
                           GetDomainOperatorService)
    services['get_domain_types'] = \
        rospy.ServiceProxy(prefix + "/get_domain_types",
                           GetDomainTypeService)
    services['planning'] = \
        rospy.ServiceProxy(prefix + "/planning_server",
                           Empty)
    services['query'] = \
        rospy.ServiceProxy(prefix + "/query_knowledge_base",
                           KnowledgeQueryService)
    services['clear_knowledge'] = \
        rospy.ServiceProxy(prefix + "/clear_knowledge_base",
                           Empty)

    db = MessageStoreProxy()


def add_instance(type_name, item_name, value=None):
    if value is not None:
        db.insert_named('%s__%s' % (type_name, item_name), value)
        types[type_name] = value.__class__._type
        # print value.__class__._type
    return services['update_knowledge_base'](KB_UPDATE_ADD_KNOWLEDGE,
        KnowledgeItem(KB_ITEM_INSTANCE,
                      type_name,
                      item_name,
                      "", [], 0.0, False))


def get_instance(type_name, item_name, return_type=None):
    if return_type is None:
        return_type = types[type_name]
    return db.query_named('%s__%s' % (type_name, item_name), return_type)


def rm_instance(type_name, item_name):
    # db[type_name].remove({'name': item_name})
    return services['update_knowledge_base'](KB_UPDATE_RM_KNOWLEDGE,
        KnowledgeItem(KB_ITEM_INSTANCE,
                      type_name,
                      item_name,
                      "", [], 0.0, False))


def list_instances(type_name, item_type=None):
    instance_names = services['get_current_instances'](type_name).instances
    if item_type:
        res = {}
        for name in instance_names:
            res[name] = get_instance(type_name, name, item_type)
        return res
    else:
        return instance_names


def is_predicate_negative(name):
    """
    Checks and remove negative symbol in case it the predicate is negated.
    The accepted format is: "[not|NOT|!] predicate_name", it is mandatory a space 
    between the negative symbol and the predicate  name.
    """

    # Check if the name is negated
    pattern_1 = "not "; pattern_2 = pattern_1.upper(); pattern_3 = "! "        
    a = (name.startswith(pattern_1) or name.startswith(pattern_2))
    b = name.startswith(pattern_3)    

    # In case it is negated, remove negative
    if a == True:
        index = len(pattern_1)
        new_name = name[index:]
    elif b == True:
        index = len(pattern_3)
        new_name = name[index:]
    else:
        new_name = name

    is_negative = (a or b)
    return new_name, is_negative


def gen_predicate(type_name, **kwargs):
    new_type_name, is_negative = is_predicate_negative(type_name)
    return KnowledgeItem(KB_ITEM_FACT,
                         "", "",
                         new_type_name,
                         dict_to_keyval(kwargs),
                         0.0, is_negative)


def add_predicate(type_name, **kwargs):
    if isinstance(type_name, KnowledgeItem):
        return services['update_knowledge_base'](KB_UPDATE_ADD_KNOWLEDGE, type_name)
    return services['update_knowledge_base'](KB_UPDATE_ADD_KNOWLEDGE,
        gen_predicate(type_name, **kwargs))


def rm_predicate(type_name, **kwargs):
    if isinstance(type_name, KnowledgeItem):
        return services['update_knowledge_base'](KB_UPDATE_RM_KNOWLEDGE, type_name)
    return services['update_knowledge_base'](KB_UPDATE_RM_KNOWLEDGE,
        gen_predicate(type_name, **kwargs))


def list_predicates():
    predicates = services['get_current_knowledge']('').attributes
    return predicates


def clear_predicates():
    for predicate in list_predicates():
        rm_predicate(predicate)


def add_goal(type_name, **kwargs):
    if isinstance(type_name, KnowledgeItem):
        return services['update_knowledge_base'](KB_UPDATE_ADD_GOAL, type_name)
    return services['update_knowledge_base'](KB_UPDATE_ADD_GOAL,
        gen_predicate(type_name, **kwargs))


def rm_goal(type_name, **kwargs):
    if isinstance(type_name, KnowledgeItem):
        return services['update_knowledge_base'](KB_UPDATE_RM_GOAL, type_name)
    return services['update_knowledge_base'](KB_UPDATE_RM_GOAL,
        gen_predicate(type_name, **kwargs))


def list_goals():
    goals = services['get_current_goals']('').attributes
    return goals


def clear_goals():
    for goal in list_goals():
        rm_goal(goal)


def get_args(item):
    global domainitems
    if item not in domainitems:
        res = services['get_domain_predicates']()
        for pred in res.items:
            domainitems[pred.name] = keyval_to_dict(pred.typed_parameters)
            if item not in domainitems:
                return False  # not in domain...

    return domainitems[item].keys()


def clear_all():
    services['clear_knowledge']()
