#! /usr/bin/python

from __future__ import absolute_import

import rospy
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService, \
    GetDomainAttributeService, GetInstanceService, GetAttributeService, \
    GetDomainOperatorService, GetDomainTypeService, KnowledgeQueryService
from std_srvs.srv import Empty
from rosplan_knowledge_msgs.msg import KnowledgeItem
from .utils import keyval_to_dict, dict_to_keyval
from mongodb_store.message_store import MessageStoreProxy

KB_UPDATE_ADD = 0
KB_UPDATE_RM = 2
KB_UPDATE_GOAL = 1
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
    services['get_current_instances_srv'] = \
        rospy.ServiceProxy(prefix + "/get_current_instances",
                           GetInstanceService)
    services['update_knowledge_base_srv'] = \
        rospy.ServiceProxy(prefix + "/update_knowledge_base",
                           KnowledgeUpdateService)
    services['get_domain_predicates_srv'] = \
        rospy.ServiceProxy(prefix + "/get_domain_predicates",
                           GetDomainAttributeService)
    services['get_current_goals_srv'] =\
        rospy.ServiceProxy(prefix + "/get_current_goals",
                           GetAttributeService)
    services['get_current_knowledge_srv'] = \
        rospy.ServiceProxy(prefix + "/get_current_knowledge",
                           GetAttributeService)
    services['get_domain_operators_srv'] = \
        rospy.ServiceProxy(prefix + "/get_domain_operators",
                           GetDomainOperatorService)
    services['get_domain_types_srv'] = \
        rospy.ServiceProxy(prefix + "/get_domain_types",
                           GetDomainTypeService)
    services['planning_srv'] = \
        rospy.ServiceProxy(prefix + "/planning_server",
                           Empty)
    services['query_srv'] = \
        rospy.ServiceProxy(prefix + "/query_knowledge_base",
                           KnowledgeQueryService)

    db = MessageStoreProxy()


def add_instance(type_name, item_name, value=None):
    if value is not None:
        db.insert_named('%s__%s' % (type_name, item_name), value)
        types[type_name] = value.__class__._type
        # print value.__class__._type
    return services['update_knowledge_base_srv'](
        KB_UPDATE_ADD,
        KnowledgeItem(KB_ITEM_INSTANCE,
                      type_name,
                      item_name,
                      "", [], 0.0))


def get_instance(type_name, item_name, return_type=None):
    if return_type is None:
        return_type = types[type_name]
    return db.query_named('%s__%s' % (type_name, item_name), return_type)


def add_predicate(type_name, *args, **kwargs):
    if args:
        args = list(args)
        for argname in get_args(type_name):
            if argname not in kwargs and args:
                kwargs[argname] = args.pop(0)
    return services['update_knowledge_base_srv'](
        KB_UPDATE_ADD,
        KnowledgeItem(KB_ITEM_FACT,
                      "", "",
                      type_name,
                      dict_to_keyval(kwargs),
                      0.0))


def rm_instance(type_name, item_name):
    # db[type_name].remove({'name': item_name})
    return services['update_knowledge_base_srv'](KB_UPDATE_RM,
                                                 KnowledgeItem(KB_ITEM_INSTANCE,
                                                               type_name,
                                                               item_name,
                                                               "", [], 0.0))


def list_instances(type_name, item_type=None):
    instance_names = services['get_current_instances_srv'](type_name).instances
    if item_type:
        res = {}
        for name in instance_names:
            res[name] = get_instance(type_name, name, item_type)
        return res
    else:
        return instance_names


def rm_predicate(type_name, *args, **kwargs):
    if args:
        args = list(args)
        for argname in get_args(type_name):
            if argname not in kwargs and args:
                kwargs[argname] = args.pop(0)
    return services['update_knowledge_base_srv'](
        KB_UPDATE_RM,
        KnowledgeItem(KB_ITEM_FACT,
                      "", "",
                      type_name,
                      dict_to_keyval(kwargs),
                      0.0))


def add_goal(type_name, *args, **kwargs):
    if args:
        args = list(args)
        for argname in get_args(type_name):
            if argname not in kwargs and args:
                kwargs[argname] = args.pop(0)
    return services['update_knowledge_base_srv'](
        KB_UPDATE_GOAL,
        KnowledgeItem(KB_ITEM_FACT,
                      "", "",
                      type_name,
                      dict_to_keyval(kwargs),
                      0.0))


def rm_goal(type_name, *args, **kwargs):
    if args:
        args = list(args)
        for argname in get_args(type_name):
            if argname not in kwargs and args:
                kwargs[argname] = args.pop(0)
    return services['update_knowledge_base_srv'](
        KB_UPDATE_RM_GOAL,
        KnowledgeItem(KB_ITEM_FACT,
                      "", "",
                      type_name,
                      dict_to_keyval(kwargs),
                      0.0))


def get_args(item):
    global domainitems
    if item not in domainitems:
        res = services['get_domain_predicates_srv']()
        for pred in res.items:
            domainitems[pred.name] = keyval_to_dict(pred.typed_parameters)
            if item not in domainitems:
                return False  # not in domain...

    return domainitems[item].keys()


def plan():
    services['planning_srv']()


if __name__ == "__main__":
    rospy.init_node("kb_demo")
    init_kb()
    add_instance('person', 'jake')
    add_predicate('robotat', 'home')
    rm_instance('person', 'jake')
    rm_predicate('robotat', 'home')
