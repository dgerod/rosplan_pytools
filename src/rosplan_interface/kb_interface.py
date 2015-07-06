#! /usr/bin/python

from __future__ import absolute_import

import rospy
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService, \
    GetDomainAttributeService
from rosplan_knowledge_msgs.msg import KnowledgeItem
from utils import keyval_to_dict, dict_to_keyval
from pymongo import MongoClient

KB_UPDATE_ADD = 0
KB_UPDATE_RM = 2
KB_UPDATE_GOAL = 1
KB_UPDATE_RM_GOAL = 3

KB_ITEM_INSTANCE = 0
KB_ITEM_FACT = 1
KB_ITEM_FUNCTION = 2

# globals must be explicit
domainitems = {}
update_srv = None
domainitem_srv = None
db = None


def init_kb(update_srv_name=None, predicate_srv_name=None):
    if update_srv_name is None:
        update_srv_name = "kcl_rosplan/update_knowledge_base"
    if predicate_srv_name is None:
        predicate_srv_name = "kcl_rosplan/get_domain_predicates"
    global update_srv
    global domainitem_srv
    global db
    update_srv = rospy.ServiceProxy(update_srv_name,
                                    KnowledgeUpdateService)
    domainitem_srv = rospy.ServiceProxy(predicate_srv_name,
                                        GetDomainAttributeService)
    rospy.wait_for_service('/message_store/update')
    # ^^^ just guarantees mongo is up before we get connection info
    mongo_host = rospy.get_param("/mongodb_host", "localhost")
    mongo_port = rospy.get_param("/mongodb_port", 27017)
    db = MongoClient(mongo_host, mongo_port)['message_store']


def add_instance(type_name, item_name, value=None):
    if value is not None:
        global db
        db[type_name].update({'name': item_name},
                             {'name': item_name, 'value': value},
                             upsert=True)
    return update_srv(KB_UPDATE_ADD, KnowledgeItem(KB_ITEM_INSTANCE,
                                                   type_name,
                                                   item_name,
                                                   "", [], 0.0))


def get_instance(type_name, item_name):
    global db
    return db[type_name].find_one({'name': item_name}).value


def add_predicate(type_name, *args, **kwargs):
    if args:
        args = list(args)
        for argname in get_args(type_name):
            if argname not in kwargs and args:
                kwargs[argname] = args.pop(0)
    return update_srv(KB_UPDATE_ADD, KnowledgeItem(KB_ITEM_FACT,
                                                   "", "",
                                                   type_name,
                                                   dict_to_keyval(kwargs),
                                                   0.0))


def rm_instance(type_name, item_name):
    db[type_name].remove({'name': item_name})
    return update_srv(KB_UPDATE_RM, KnowledgeItem(KB_ITEM_INSTANCE,
                                                  type_name,
                                                  item_name,
                                                  "", [], 0.0))


def rm_predicate(type_name, *args, **kwargs):
    if args:
        args = list(args)
        for argname in get_args(type_name):
            if argname not in kwargs and args:
                kwargs[argname] = args.pop(0)
    return update_srv(KB_UPDATE_RM, KnowledgeItem(KB_ITEM_FACT,
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
    return update_srv(KB_UPDATE_GOAL, KnowledgeItem(KB_ITEM_FACT,
                                                    "", "",
                                                    type_name,
                                                    dict_to_keyval(kwargs),
                                                    0.0))


def get_args(item):
    global domainitems

    if item not in domainitems:
        res = domainitem_srv()
        for pred in res.items:
            domainitems[pred.name] = keyval_to_dict(pred.typed_parameters)
    if item not in domainitems:
        return False  # not in domain...

    return domainitems[item].keys()


if __name__ == "__main__":
    rospy.init_node("kb_demo")
    init()
    add_instance('person', 'jake')
    add_predicate('robotat', 'home')
    rm_instance('person', 'jake')
    rm_predicate('robotat', 'home')
