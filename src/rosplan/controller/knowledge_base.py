"""
 ROSPlan Knowledge Base interface
 Avoids the totally convoluted syntax of ROSPlan, and lets you easily put data in
 the scene database at the same time.
"""

from __future__ import absolute_import
from collections import namedtuple

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

QueryResult = namedtuple('QueryResult', 'success item')

_services = {} # kb
_sdb = None
_local_storage_item_types = {}
_local_storage_item_domains = {}


class Item(object):

    def __init__(self, name="", category="", value=None):
        self._name = name
        self._category = category
        self._value = value

        self._instance = None

    def __str__(self):

        value = "None"
        if self._value is not None:
            value = self._value

        instance = "None"
        if self._instance is not None:
            instance = self._instance

        return "%s, %s, %s; %s" % \
               (self._name, self._category, value, instance)

    def is_valid(self):
        return self._name != ""

    def name(self):
        return self._name

    def category(self):
        return self._category

    def value(self):
        return self._value

    def set_value(self, value):
        self._value = value
        self._instance = None

    def get_type(self):
        if self._value is not None:
            return self._value.__class__._type
        else:
            return None

    def _update(self, instance):
        self._value = instance[0]
        self._instance = instance

    def _get_id(self):
        return self._instance[1]["_id"]

    def _get_ros_message(self):
        return self._instance[0]

    def _get_meta_info(self):
        return self._instance[1]


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


def _initialize_sdb_connection():
    global _sdb
    _sdb = MessageStoreProxy()


def _register_item_type(category, type_):
    global _local_storage_item_types
    if category not in _local_storage_item_types:
        _local_storage_item_types[category] = type_


def _get_item_type(category):

    global _local_storage_item_types
    type_ = None

    if category in _local_storage_item_types:
        type_ = _local_storage_item_types[category]

    return type_


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


def _instance_exists(item_name):
    instance_names = _services["get_current_instances"]("").instances
    return item_name in instance_names


def _query_item_by_name_and_category(item_name, item_category, item_type):
    """
    :param item_name: name of the item (str)
    :param item_category: name of the item category (str)
    :param item_type: class that defines an item value (ros message)
    :return: Item
    """

    # Search item type in local storage
    if item_type is None:
        item_type = _get_item_type(item_category)
        if item_type is not None:
            return Item()

    item = Item()

    # Example in MongoDB:
    #  db.getCollection('message_store').find({'_meta.name': 'waypoint__p1'})
    instance = _sdb.query_named("%s__%s" % (item_category, item_name), item_type)
    if instance[0] is not None:
        item = Item(item_name, item_category)
        item._update(instance)

    return item


def _query_item_by_name(item_name):
    """
    :param item_name: name of the item (str)
    :return: Item
    """

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

    results = _services["get_domain_types"]()

    item_types = {}
    for item_category in results.types:
        item_types[item_category] = _get_item_type(item_category)

    item = Item()
    for item_category in results.types:

        instance = _sdb.query_named("%s__%s" % (item_category, item_name), item_types[item_category])
        if instance[0] is not None:
            item = Item(item_name, item_category)
            item._update(instance)
            break

    return item


def _find_instance(item, item_type):
    """
    :param item: Item, that's name, category, etc.
    :param item_type: class that defines an item value (ros message)
    :return: success and Item
    """

    if not _instance_exists(item.name()):
        return False, Item()

    item_name = item.name()
    item_category = item.category()

    if item_category != "":

        if item_type is None:
            item_type = item.get_type()

        registered_item = _query_item_by_name_and_category(item_name, item_category, item_type)

    else:
        registered_item = _query_item_by_name(item_name)

    return registered_item.is_valid(), registered_item


def init(prefix=None):

    if prefix is None:
        prefix = "/kcl_rosplan"

    _initialize_kb_services(prefix)
    _initialize_sdb_connection()


def exist_instance(item):
    """
    :param item: name of the item (str) or an item instance (Item)
    :return: success

    Usage:
        success = add_instance(Item) --> Item instance
        success = add_instance(str) --> item as string
    """

    if isinstance(item, str):
        new_item = Item(item)
    else:
        new_item = item

    return _instance_exists(new_item)

This work is ongoing, it must be tested before if completed. And after testing check how
to unify add_instance(Item) with other functions as add_predicate(...)

Need to test:
    add_instance(), get_instance() and other functions
    it seems that it is failing because item type is not correctly got from local storage.

def add_instance(item, category="", value=None):
    """
    :param item: name of the item (str) or an item instance (Item)
    :param value: a class with the value of the item (ros message)
    :param category: name of the item type (str)
    :return: QueryResult(success, Item)

    Usage:
        success, Item = add_instance(Item) --> Item instance that could include value
        success, Item = add_instance(str, str) --> item, category as strings without value
        success, Item = add_instance(str, str, object) --> item, category as strings and value
    """

    if (isinstance(item, str) and category == "") or \
            (not isinstance(item, str) and (category != "" or value is not None)):
        return QueryResult(False, Item())

    if isinstance(item, str):
            new_item = Item(item, category)
            new_item.set_value(value)
    else:
            new_item = item

    success = _services["update_knowledge_base"](KB_UPDATE_ADD_KNOWLEDGE,
                                                 KnowledgeItem(KB_ITEM_INSTANCE,
                                                               new_item.category(),
                                                               new_item.name(),
                                                               "", [], 0.0, False))

    if success and new_item.value() is not None:

        _sdb.insert_named("%s__%s" % (new_item.category(), new_item.name()), new_item.value())
        _register_item_type(new_item.category(), new_item.get_type())

        success, updated_item = _find_instance(new_item, None)
        new_item = updated_item

    return QueryResult(success, new_item)


def update_instance(item, value=None):
    """
    :param item: name of the item (str) or an item instance (Item)
    :param value: a class with the value of the item (ros message) to be updated
    :return: QueryResult(success, Item)

    Usage:
        success, Item = update_instance(Item, object) --> Item instance and new value
        success, Item = update_instance(str, object) --> item as string and new value
    """

    if (isinstance(item, str) and value is None) or\
            (not isinstance(item, str) and value is not None):
        return QueryResult(False, Item())

    if isinstance(item, str):
        new_item = Item(item, "")
        new_item.set_value(value)
    else:
        new_item = item

    success, registered_item = _find_instance(new_item, None)
    if registered_item.is_valid():

        _sdb.update_named(registered_item.name(), new_item.value())
        success, updated_item = _find_instance(new_item, None)

        return QueryResult(success, updated_item)

    else:
        return QueryResult(False, Item())


def get_instance(item, category="", item_type=None):
    """
    :param item: name of the item (str) or an item instance (Item)
    :param category: name of the item type (str)
    :param item_type: class that defines an item value (ros message)
    :return: QueryResult(success, Item)

    Usage:
        success, Item = get_instance(Item) --> Item instance
        success, Item = get_instance(str, str, object) --> item and category as strings and value type
        success, Item = get_instance(str, str) --> Item instance
    """

    if (isinstance(item, str) and category == "") or \
            (not isinstance(item, str) and (category != "" or item_type is not None)):
        return QueryResult(False, Item())

    if isinstance(item, str):
        new_item = Item(item, category)
    else:
        new_item = item

    success, registered_item = _find_instance(new_item, item_type)
    return QueryResult(success, registered_item)


def rm_instance(item, category=""):
    """
    :param item: name of the item (str) or an item instance (Item)
    :param category: name of the item type (str)
    :return: success

    Usage:
        success = rm_instance(Item) --> Item instance
        success = rm_instance(str, str) --> item and category as strings and value type
    """

    if (isinstance(item, str) and category == "") or \
            (not isinstance(item, str) and category != ""):
        return False

    if isinstance(item, str):
        new_item = Item(item)
    else:
        new_item = item

        success, registered_item = _find_instance(new_item, None)
        if success:
            _sdb.delete(str(registered_item._get_id()))

    return _services["update_knowledge_base"](KB_UPDATE_RM_KNOWLEDGE,
                                              KnowledgeItem(KB_ITEM_INSTANCE,
                                                            new_item.category(),
                                                            new_item.name(),
                                                            "", [], 0.0, False))


def list_instances(item_category="", item_type=None):
    instance_names = _services["get_current_instances"](item_category).instances
    if item_type:
        res = {}
        for item_name in instance_names:
            res[item_name] = get_instance(item_name, item_category, item_type)
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
    global _local_storage_item_domains
    if item not in _local_storage_item_domains:
        res = _services["get_domain_predicates"]()
        for predicate in res.items:
            _local_storage_item_domains[predicate.name] = keyval_to_dict(predicate.typed_parameters)
            if item not in _local_storage_item_domains:
                return False  # not in domain...
    return _local_storage_item_domains[item].keys()


def clear_all():
    _services["clear_knowledge"]()
    # After cleaning the KB we must clean all information in the SDB
    # system("mongo message_store --eval \"printjson(db.message_store.remove({}))\"");
