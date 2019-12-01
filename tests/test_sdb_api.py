#!/usr/bin/env python

import unittest
import mock

import rosplan_pytools.controller.scene_database as sdb
from std_msgs.msg import Empty


class TestSceneDatabase(unittest.TestCase):

    @mock.patch('rosplan_pytools.controller.scene_database.rospy.has_param')
    @mock.patch('rosplan_pytools.controller.scene_database.rospy.get_param')
    @mock.patch('rosplan_pytools.controller.scene_database.rospy.set_param')
    def test_element_exists(self, mock_set_param, mock_get_param, mock_has_param):
        pass


class TestSceneDatabaseWithRealConnection(unittest.TestCase):

    def test_add_and_remove(self,):

        sdb.initialize()
        sdb.reset()

        self.assertEqual(sdb.num_elements(), 0)

        print(sdb.list_elements())

        self.assertTrue(sdb.add_element("key_3", sdb.Element(Empty())))
        self.assertEqual(sdb.num_elements(), 1)

        print(sdb.list_elements())

        self.assertTrue(sdb.add_element("key_2", sdb.Element(Empty())))
        self.assertEqual(sdb.num_elements(), 2)

        print(sdb.list_elements())

        self.assertTrue(sdb.add_element("key_1", sdb.Element(Empty())))
        self.assertEqual(sdb.num_elements(), 3)

        print(sdb.list_elements())

        self.assertTrue(sdb.remove_element("key_3"))
        self.assertEqual(sdb.num_elements(), 2)

        print(sdb.list_elements())

        self.assertTrue(sdb.remove_element("key_2"))
        self.assertEqual(sdb.num_elements(), 1)

        print(sdb.list_elements())

        self.assertTrue(sdb.remove_element("key_1"))
        self.assertEqual(sdb.num_elements(), 0)

        print(sdb.list_elements())


if __name__ == '__main__':
    unittest.main()
