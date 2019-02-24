#!/usr/bin/env python

from collections import namedtuple

import unittest

from rosplan_pytools.controller import knowledge_base as kb
from geometry_msgs.msg import Pose


class LocalKB(object):

    _items = []
    _last_operation_result = False

    def __init__(self):

        self._kb_services = dict()
        self._kb_services["get_current_instances"] = LocalKB._get_current_instances
        self._kb_services["get_domain_types"] = LocalKB._get_domain_types
        self._kb_services["update_knowledge_base"] = LocalKB._update_knowledge_base

        class SDB(object):
            def insert_named(self, key, value):
                pass
            def query_named(self, key, type_):
                pass

        self._sdb = SDB()
        self._sdb.insert_named = LocalKB._insert_named
        self._sdb.query_named = LocalKB._query_named

    @classmethod
    def init(cls):
        cls._items = []
        cls._last_operation_result = False
        return LocalKB()

    def get_kb(self):
        return self._kb_services, self._sdb

    @classmethod
    def add_instances(cls, instances):
        for name in instances:
            if name not in cls._items:
                cls._items.append({"name": name, "category": None, "type": None, "instance": None})

    @classmethod
    def add_item(cls, name, category, type_, instance):
        if name not in cls._items:
            item = {"name": name, "category": category, "type": type_, "instance": instance}
            cls._items.append(item)

    @classmethod
    def set_operation_result(cls, succeed):
        cls._last_operation_result = succeed

    def get_item(self, name):
        return self._search_item_by_name(name)

    @staticmethod
    def _search_item_by_name(name):
        for item in LocalKB._items:
            if name in item['name']:
                return item
        return None

    # Mocking KS services and SDB methods
    # -----------------------------------------------------------

    @staticmethod
    def _get_current_instances(category):
        from collections import namedtuple
        ServiceResponse = namedtuple("ServiceResponse", "instances")

        instances = []
        for item in LocalKB._items:
            if category == "" or (category != "" and item["category"] == category):
                instances.append(item["name"])

        return ServiceResponse(instances)

    @staticmethod
    def _get_domain_types():
        return None

    @staticmethod
    def _update_knowledge_base(operation, item):
        return LocalKB._last_operation_result

    @staticmethod
    def _insert_named(key, value):
        category = key.split("__", 1)[0]
        name = key.split("__", 1)[1]
        if LocalKB._search_item_by_name(name) is None:
            LocalKB.add_item(name, category, value.__class__._type, [value, None])
            return True
        else:
            return False

    @staticmethod
    def _query_named(key, type_):
        name = key.split("__", 1)[1] # category__name
        item = LocalKB._search_item_by_name(name)
        return item['instance']


class TestExistInstance(unittest.TestCase):

    def setUp(self):
        self._local_kb = LocalKB.init()
        kb._initialize_local_storage()

    def tearDown(self):
        pass

    def test_1_instance_not_exists(self):

        kb._services, kb._sdb = self._local_kb.get_kb()
        self._local_kb.add_instances(['v1', 'v2'])

        # Using new interface: Item class
        # -------------------------------------------------------------

        success = kb.exist_instance(kb.Item('v100'))
        self.assertFalse(success, 'Instance does not exist')

        # Using legacy interface: name, category, value
        # -------------------------------------------------------------

        success = kb.exist_instance('v200')
        self.assertFalse(success, 'Instance does not exist')

    def test_2_instance_exists(self):

        kb._services, kb._sdb = self._local_kb.get_kb()
        self._local_kb.add_instances(['v100', 'v200'])

        # Using new interface: Item class
        # -------------------------------------------------------------

        success = kb.exist_instance(kb.Item('v100'))
        self.assertTrue(success, 'Instance exists')

        # Using legacy interface: name, category, value
        # -------------------------------------------------------------

        success = kb.exist_instance('v200')
        self.assertTrue(success, 'Instance exists')


class TestListInstances(unittest.TestCase):

    def setUp(self):
        self._local_kb = LocalKB.init()
        kb._initialize_local_storage()

    def tearDown(self):
        pass

    def test_all_instances(self):
        kb._services, kb._sdb = self._local_kb.get_kb()

        self._local_kb.add_item('v1', 'videotape', None, None)
        self._local_kb.add_item('v2', 'videotape', None, None)
        self._local_kb.add_item('c', 'container', None, None)
        self._local_kb.add_item('b1', 'box', None, None)
        self._local_kb.add_item('b2', 'box', None, None)

        instances = kb.list_instances()

        success = True
        for instance in instances:
            if instance not in ['v1','v2','c','b1','b2']:
                success = False

        self.assertTrue(success, 'Returned correct instances')

    def test_instances_by_category(self):

        kb._services, kb._sdb = self._local_kb.get_kb()

        self._local_kb.add_item('v1', 'videotape', None, None)
        self._local_kb.add_item('v2', 'videotape', None, None)
        self._local_kb.add_item('c', 'container', None, None)
        self._local_kb.add_item('b1', 'box', None, None)
        self._local_kb.add_item('b2', 'box', None, None)

        instances = kb.list_instances('box')

        success = True
        for instance in instances:
            if instance not in ['b1','b2']:
                success = False

        self.assertTrue(success, 'Returned correct instances')

    def test_instances_by_type(self):
        pass


class TestAddInstance(unittest.TestCase):

    def setUp(self):
        self._local_kb = LocalKB.init()
        kb._initialize_local_storage()

    def tearDown(self):
        pass

    def test_1_check_incorrect_inputs(self):

        # Using new interface: Item class
        # -------------------------------------------------------------

        item = kb.Item('v100', 'videotape')

        result = kb.add_instance(item, 'videotape', 'value')
        self.assertFalse(result.success, 'When using Item other arguments could not be used')

        result = kb.add_instance(item, 'videotape')
        self.assertFalse(result.success, 'When using Item other arguments could not be used')

        result = kb.add_instance(item, '', 'value')
        self.assertFalse(result.success, 'When using Item other arguments could not be used')

        # Using legacy interface: name, category, value
        # -------------------------------------------------------------

        result = kb.add_instance('v200', '')
        self.assertFalse(result.success, 'When using name category is mandatory but value is optional')

        result = kb.add_instance('v200', '')
        self.assertFalse(result.success, 'When using name category is mandatory but value is optional')

        result = kb.add_instance('v200', '', 'value')
        self.assertFalse(result.success, 'When using name category is mandatory but value is optional')

    def test_2_instance_without_value(self):

        kb._services, kb._sdb = self._local_kb.get_kb()

        # Using new interface: Item class
        # -------------------------------------------------------------

        self._local_kb.set_operation_result(False)

        item = kb.Item('v100', 'videotape')

        result = kb.add_instance(item)
        self.assertFalse(result.success, 'Failure updating KB')

        self._local_kb.set_operation_result(True)

        item = kb.Item('v100', 'videotape')
        result = kb.add_instance(item)
        self.assertTrue(result.success, 'KB is updated correctly')
        self.assertEqual(result.item.name(), 'v100')
        self.assertEqual(result.item.category(), 'videotape')

        # Using legacy interface: name, category, value
        # -------------------------------------------------------------

        self._local_kb.set_operation_result(False)

        result = kb.add_instance('v200', 'videotape')
        self.assertFalse(result.success, 'Failure updating KB')

        self._local_kb.set_operation_result(True)

        result = kb.add_instance('v200', 'videotape')
        self.assertTrue(result.success, 'KB is updated correctly')
        self.assertEqual(result.item.name(), 'v200')
        self.assertEqual(result.item.category(), 'videotape')

    def test_3_instance_with_value(self):

        kb._services, kb._sdb = self._local_kb.get_kb()

        # Using new interface: Item class
        # -------------------------------------------------------------

        self._local_kb.set_operation_result(False)

        item = kb.Item('v100', 'videotape', Pose())
        result = kb.add_instance(item)

        self.assertFalse(result.success, 'Failure updating KB')
        self.assertEqual(result.item.name(), '')
        self.assertEqual(result.item.category(), '')
        self.assertEqual(result.item.get_ros_type(), None)

        self._local_kb.set_operation_result(True)

        item = kb.Item('v100', 'videotape', Pose())
        result = kb.add_instance(item)

        self.assertTrue(result.success, 'KB is updated correctly')
        self.assertEqual(result.item.name(), 'v100')
        self.assertEqual(result.item.category(), 'videotape')

        # Using legacy interface: name, category, value
        # -------------------------------------------------------------

        self._local_kb.set_operation_result(False)

        result = kb.add_instance('v200', 'videotape', Pose())

        self.assertFalse(result.success, 'Failure updating KB')
        self.assertEqual(result.item.name(), '')
        self.assertEqual(result.item.category(), '')
        self.assertEqual(result.item.get_ros_type(), None)

        self._local_kb.set_operation_result(True)

        result = kb.add_instance('v201', 'videotape', Pose())

        self.assertTrue(result.success, 'KB is updated correctly')
        self.assertEqual(result.item.name(), 'v201')
        self.assertEqual(result.item.category(), 'videotape')
        self.assertEqual(result.item.get_ros_type(), Pose().__class__._type)


class TestGetInstance(unittest.TestCase):
    def setUp(self):
        self._local_kb = LocalKB.init()
        kb._initialize_local_storage()

    def tearDown(self):
        pass

    def test_1_check_incorrect_inputs(self):

        kb._services, kb._sdb = self._local_kb.get_kb()

        # Using new interface: Item class
        # -------------------------------------------------------------

        item = kb.Item('v100')

        result = kb.get_instance(item, "category", "value")
        self.assertFalse(result.success, 'When using Item other arguments could not be used')

        result = kb.get_instance(item, "", "value")
        self.assertFalse(result.success, 'When using Item other arguments could not be used')

        result = kb.get_instance(item, "category")
        self.assertFalse(result.success, 'When using Item other arguments could not be used')

        # Using legacy interface: name, category, value
        # -------------------------------------------------------------

        #result = kb.get_instance('v200', "")
        #self.assertFalse(result.success, 'When using name value could not be empty')

        #result = kb.get_instance('v200', "", "value")
        #self.assertFalse(result.success, 'When using name value could not be empty')

    def test_2_fails_as_type_is_unknown(self):
        kb._services, kb._sdb = self._local_kb.get_kb()
        self._local_kb.add_instances(['v1', 'v2'])

        item = kb.Item('v1', 'videotape')
        success, instance = kb.get_instance(item)

        self.assertFalse(success, 'It fails as type is not defined')

    def test_4_fails_as_returned_instance_is_unknown(self):
        kb._services, kb._sdb = self._local_kb.get_kb()
        self._local_kb.add_item('v1', 'videotape', Pose(), [None, None])

        kb.register_type('videotape', Pose().__class__._type)

        item = kb.Item('v1', 'videotape')
        success, instance = kb.get_instance(item)

        self.assertFalse(success, 'It fails as instance is not returned')

    def test_5_find_instance_succeed(self):
        kb._services, kb._sdb = self._local_kb.get_kb()

        p = Pose()
        self._local_kb.add_item('v1', 'videotape', None, [p, None])

        kb.register_type('videotape', Pose())

        item = kb.Item('v1', 'videotape')
        success, instance = kb.get_instance(item)

        self.assertTrue(success, 'Find succeeded')

    def test_2_check_incorrect_inputs(self):

        kb._services, kb._sdb = self._local_kb.get_kb()
        self._local_kb.add_item('v100','videotape', None, [Pose(), None])

        kb.register_type('videotape', Pose())

        item = kb.Item('v100','videotape')
        result = kb.get_instance(item)
        self.assertTrue(result.success, 'Returning existing instance')
        self.assertEqual(result.item.name(), 'v100')
        self.assertEqual(result.item.category(), 'videotape')
        self.assertEqual(result.item.get_ros_type(), Pose().__class__._type)


class TestUpdateInstance(unittest.TestCase):

    def setUp(self):
        self._local_kb = LocalKB.init()
        kb._initialize_local_storage()

    def tearDown(self):
        pass

    def test_1_check_incorrect_inputs(self):

        kb._services, kb._sdb = self._local_kb.get_kb()

        # Using new interface: Item class
        # -------------------------------------------------------------

        item = kb.Item('v100')
        result = kb.update_instance(item, Pose())
        self.assertFalse(result.success, 'When using Item other arguments could not be used')

        # Using legacy interface: name, category, value
        # -------------------------------------------------------------

        result = kb.update_instance('v100', None)
        self.assertFalse(result.success, 'When using name value could not be empty')


class TestRemoveInstance(unittest.TestCase):
    pass


# Following test must be converted to get_instance as we have to test public interfaces.
class TestFindInstance(unittest.TestCase):

    def setUp(self):
        self._local_kb = LocalKB.init()
        kb._initialize_local_storage()

    def tearDown(self):
        pass

    def test_1_instance_not_exists(self):

        kb._services, kb._sdb = self._local_kb.get_kb()
        self._local_kb.add_instances(['v1', 'v2'])

        item = kb.Item('v100', 'videotape')
        success, instance = kb._find_instance(item)

        self.assertFalse(success, 'Instance does not exist in the KB')

    def test_2_fails_as_type_is_unknown(self):

        kb._services, kb._sdb = self._local_kb.get_kb()
        self._local_kb.add_instances(['v1', 'v2'])

        item = kb.Item('v1', 'videotape')
        success, instance = kb._find_instance(item)

        self.assertFalse(success, 'It fails as type is not defined')

    def test_4_fails_as_returned_instance_is_unknown(self):

        kb._services, kb._sdb = self._local_kb.get_kb()
        self._local_kb.add_item('v1', 'videotape', Pose(), [None, None])

        kb.register_type('videotape', Pose().__class__._type)

        item = kb.Item('v1', 'videotape')
        success, instance = kb._find_instance(item)

        self.assertFalse(success, 'It fails as instance is not returned')

    def test_5_find_instance_succeed(self):

        kb._services, kb._sdb = self._local_kb.get_kb()

        p = Pose()
        self._local_kb.add_item('v1', 'videotape', None, [p, None])

        kb.register_type('videotape', Pose())

        item = kb.Item('v1', 'videotape')
        success, instance = kb._find_instance(item)

        self.assertTrue(success, 'Find succeeded')


if __name__ == '__main__':
    unittest.main()
