"""
 ROSPlan Knowledge Base interface
 Avoids the totally convoluted syntax of ROSPlan, and lets you easily put data in
 the scene database at the same time.
"""

from __future__ import absolute_import

import rospy
from std_srvs.srv import Empty
from mongodb_store.message_store import MessageStoreProxy
from rosplan_knowledge_msgs.srv import \
    KnowledgeUpdateService, KnowledgeQueryService, \
    GetInstanceService, GetAttributeService, GetDomainAttributeService, \
    GetDomainTypeService, GetDomainOperatorService, \
    GetDomainOperatorDetailsService, GetDomainPredicateDetailsService
from rosplan_knowledge_msgs.msg import KnowledgeItem
from rosplan.common.utils import keyval_to_dict, dict_to_keyval

KB_UPDATE_ADD_KNOWLEDGE = 0
KB_UPDATE_RM_KNOWLEDGE = 2
KB_UPDATE_ADD_GOAL = 1
KB_UPDATE_RM_GOAL = 3

KB_ITEM_INSTANCE = 0
KB_ITEM_FACT = 1
KB_ITEM_FUNCTION = 2

_services = {}
_sdb = None
_value_types = {}
_domain_items = {}


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


def _gen_predicate(type_name, **kwargs):
    new_type_name, is_negative = _is_predicate_negative(type_name)
    return KnowledgeItem(KB_ITEM_FACT,
                         "",
                         "",
                         new_type_name,
                         dict_to_keyval(kwargs),
                         0.0, is_negative)


def _find_instance(item_name, type_name=None, value_type=None):

    # Check if item exists in the KB
    instance_names = _services["get_current_instances"]("").instances
    if (item_name in instance_names) is False:
        return None, None

    # In case value type is not set find in the local storage.
    if value_type is None and type_name is not None:
        if _value_types[type_name] is not None:
            value_type = _value_types[type_name]
        else:
            return None, None

    # Find data associated to the item in the SDB
    if (type_name is not None) and (type_name != ""):
        # Example in MongoDB:
        #   db.getCollection('message_store').find({'_meta.name': 'waypoint__p1'})
        instance = _sdb.query_named("%s__%s" % (type_name, item_name), value_type)
    else:
        # This is not the best approach because it makes a lot of queries to
        # the db. The query in MongoDB is something like:
        #   db.getCollection('message_store').find({'_meta.name': {$regex: /p1/}})
        #
        # And here the code should be something like:
        #   meta = {}
        #   meta['name'] = '{$regex: /p1/}'
        #   p1 = db.query(Pose._type, {}, meta, True, [], {}, 0)
        #   print "db.query 3\n", p1
        #
        # Unfortunately, this code is not working as I am not able to pass a regex
        # instruction to MongoDB using 'mongodb_store'. Therefore, I used a workaround
        # that implies getting types in domain and use them to get the instance.
        res = _services["get_domain_types"]()
        for type_name in res.types:
            instance = _sdb.query_named("%s__%s" % (type_name, item_name), value_type)
            if instance is not None:
                break

    return instance, type_name


def init(prefix=None):
    if prefix is None:
        prefix = "/kcl_rosplan"

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

    global _sdb
    _sdb = MessageStoreProxy()


def exist_instance(item_name):
    instance_names = _services["get_current_instances"]("").instances
    return item_name in instance_names


def add_instance(item_name, type_name, value=None):
    if value is not None:
        _sdb.insert_named("%s__%s" % (type_name, item_name), value)
        _value_types[type_name] = value.__class__._type
        # print value.__class__._type
    return _services["update_knowledge_base"](KB_UPDATE_ADD_KNOWLEDGE,
                                              KnowledgeItem(KB_ITEM_INSTANCE,
                                                            type_name,
                                                            item_name,
                                                            "", [], 0.0, False))


def update_instance(item_name, value):
    instance, type_name = _find_instance(item_name, None, None)

    if instance is not None:
        _sdb.update_named(item_name, value)
        return True
    else:
        return False


def get_instance(item_name, type_name=None, value_type=None):
    return _find_instance(item_name, type_name, value_type)


def rm_instance(item_name, type_name):
    return _services["update_knowledge_base"](KB_UPDATE_RM_KNOWLEDGE,
                                              KnowledgeItem(KB_ITEM_INSTANCE,
                                                            type_name,
                                                            item_name,
                                                            "", [], 0.0, False))


def list_instances(type_name="", item_type=None):
    instance_names = _services["get_current_instances"](type_name).instances
    if item_type:
        res = {}
        for item_name in instance_names:
            res[item_name] = get_instance(item_name, type_name, item_type)
        return res
    else:
        return instance_names


# def clear_instances():
#        for item_name in list_instances():
#            instance, type_name = get_instance(None, item_name)
#            rm_instance(type_name, item_name)


def add_predicate(type_name, **kwargs):
    if isinstance(type_name, KnowledgeItem):
        return _services["update_knowledge_base"](KB_UPDATE_ADD_KNOWLEDGE, type_name)
    return _services["update_knowledge_base"](KB_UPDATE_ADD_KNOWLEDGE,
                                              _gen_predicate(type_name, **kwargs))


def rm_predicate(type_name, **kwargs):
    if isinstance(type_name, KnowledgeItem):
        return _services["update_knowledge_base"](KB_UPDATE_RM_KNOWLEDGE, type_name)
    return _services["update_knowledge_base"](KB_UPDATE_RM_KNOWLEDGE,
                                              _gen_predicate(type_name, **kwargs))


def list_predicates():
    predicates = _services["get_current_knowledge"]("").attributes
    return predicates


def clear_predicates():
    for predicate in list_predicates():
        rm_predicate(predicate)


def add_goal(type_name, **kwargs):
    if isinstance(type_name, KnowledgeItem):
        return _services["update_knowledge_base"](KB_UPDATE_ADD_GOAL, type_name)
    return _services["update_knowledge_base"](KB_UPDATE_ADD_GOAL,
                                              _gen_predicate(type_name, **kwargs))


def rm_goal(type_name, **kwargs):
    if isinstance(type_name, KnowledgeItem):
        return _services["update_knowledge_base"](KB_UPDATE_RM_GOAL, type_name)
    return _services["update_knowledge_base"](KB_UPDATE_RM_GOAL,
                                              _gen_predicate(type_name, **kwargs))


def list_goals():
    goals = _services["get_current_goals"]("").attributes
    return goals


def clear_goals():
    for goal in list_goals():
        rm_goal(goal)


def get_args(item):
    global _domain_items
    if item not in _domain_items:
        res = _services["get_domain_predicates"]()
        for predicate in res.items:
            _domain_items[predicate.name] = keyval_to_dict(predicate.typed_parameters)
            if item not in _domain_items:
                return False  # not in domain...
    return _domain_items[item].keys()


def clear_all():
    _services["clear_knowledge"]()
    # After cleaning the KB we must clean all information in the SDB
