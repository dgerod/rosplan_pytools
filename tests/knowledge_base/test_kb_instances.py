#!/usr/bin/env python

import unittest
import mock

import rosplan_pytools.rosplan.controller.knowledge_base as kb

from rosplan_knowledge_msgs.srv import GetInstanceServiceResponse
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceResponse


class TestListInstances(unittest.TestCase):

    def test_add_instance_incorrect_arguments(self):

        self.assertRaises(TypeError, lambda: kb.add_instance(10.0, ''))
        self.assertRaises(TypeError, lambda: kb.add_instance('', 10.0))

    @mock.patch('rosplan_pytools.controller.knowledge_base._services')
    @mock.patch('rosplan_pytools.controller.knowledge_base._initialize_kb_services')
    def test_add_instance(self, mock_initialize_kb_service, mock_services):

        kb.initialize()

        def _update_knowledge_base(kb_operation, kb_item):
            return KnowledgeUpdateServiceResponse(True)

        services = {"update_knowledge_base": _update_knowledge_base}
        mock_services.__getitem__.side_effect = lambda key: services[key]

        self.assertTrue(kb.add_instance('v1', 'videotape'))
        self.assertTrue(kb.add_instance('v2', 'videotape'))
        self.assertTrue(kb.add_instance('c', 'container'))
        self.assertTrue(kb.add_instance('b1', 'box'))
        self.assertTrue(kb.add_instance('b2', 'box'))

        def _get_current_instances(type_name):
            return GetInstanceServiceResponse(['v1', 'v2', 'c', 'b1', 'b2'])

        services = {"get_current_instances": _get_current_instances}
        mock_services.__getitem__.side_effect = lambda key: services[key]

        instances = kb.list_instances()

        success = True
        for instance in instances:
            if instance not in ['v1', 'v2', 'c', 'b1', 'b2']:
                success = False

        self.assertTrue(success, 'Returned not correct instances')

    @mock.patch('rosplan_pytools.controller.knowledge_base._services')
    @mock.patch('rosplan_pytools.controller.knowledge_base._initialize_kb_services')
    def test_remove_instance(self, mock_initialize_kb_service, mock_services):
        pass

    def test_remove_all_instances_not_implemented(self):

        self.assertRaises(NotImplementedError, lambda: kb.remove_all_instances())

    @mock.patch('rosplan_pytools.controller.knowledge_base._services')
    @mock.patch('rosplan_pytools.controller.knowledge_base._initialize_kb_services')
    def test_add_predicate(self, mock_initialize_kb_service, mock_services):
        pass

    @mock.patch('rosplan_pytools.controller.knowledge_base._services')
    @mock.patch('rosplan_pytools.controller.knowledge_base._initialize_kb_services')
    def test_remove_predicate(self, mock_initialize_kb_service, mock_services):
        pass

    @mock.patch('rosplan_pytools.controller.knowledge_base._services')
    @mock.patch('rosplan_pytools.controller.knowledge_base._initialize_kb_services')
    def test_add_goal(self, mock_initialize_kb_service, mock_services):
        pass

    @mock.patch('rosplan_pytools.controller.knowledge_base._services')
    @mock.patch('rosplan_pytools.controller.knowledge_base._initialize_kb_services')
    def test_remove_goal(self, mock_initialize_kb_service, mock_services):
        pass


if __name__ == '__main__':
    unittest.main()
