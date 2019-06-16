#!/usr/bin/env python

import unittest
import mock
import rosplan_pytools.controller.scene_database as sdb
from rosplan_pytools.common import message_converter

from std_msgs.msg import Empty
from geometry_msgs.msg import Pose


class TestRosServerConnection(unittest.TestCase):

    @mock.patch('rosplan_pytools.controller.scene_database.rospy.has_param')
    @mock.patch('rosplan_pytools.controller.scene_database.rospy.get_param')
    @mock.patch('rosplan_pytools.controller.scene_database.rospy.set_param')
    def test_connect_when_db_does_not_exist(self, mock_set_param, mock_get_param, mock_has_param):

        mock_has_param.side_effect = [False]
        mock_set_param.side_effect = [True]
        mock_get_param.side_effect = [0, 0]

        ros_server = sdb._RosServerConnection()

        self.assertEqual(mock_has_param.call_count, 1)
        self.assertEqual(mock_set_param.call_count, 1)
        self.assertEqual(ros_server.num_elements(), 0)

        # mock.assert_called_with(1, 2, value="alpha")

    @mock.patch('rosplan_pytools.controller.scene_database.rospy.has_param')
    @mock.patch('rosplan_pytools.controller.scene_database.rospy.get_param')
    @mock.patch('rosplan_pytools.controller.scene_database.rospy.set_param')
    def test_connect_when_db_exists(self, mock_set_param, mock_get_param, mock_has_param):

        mock_has_param.side_effect = [True]
        mock_set_param.side_effect = [True]
        mock_get_param.side_effect = [0]

        ros_server = sdb._RosServerConnection()

        self.assertEqual(mock_has_param.call_count, 1)
        self.assertEqual(mock_set_param.call_count, 0)
        self.assertEqual(ros_server.num_elements(), 0)

    @mock.patch('rosplan_pytools.controller.scene_database.rospy.has_param')
    @mock.patch('rosplan_pytools.controller.scene_database.rospy.get_param')
    @mock.patch('rosplan_pytools.controller.scene_database.rospy.set_param')
    def test_add_new_element_in_empty_db(self, mock_set_param, mock_get_param, mock_has_param):

        mock_has_param.side_effect = [False]
        mock_set_param.side_effect = [True]
        mock_get_param.side_effect = [0]

        ros_server = sdb._RosServerConnection()

        self.assertEqual(mock_has_param.call_count, 1)
        self.assertEqual(mock_set_param.call_count, 1)
        self.assertEqual(ros_server.num_elements(), 0)

        mock_has_param.side_effect = [False]
        mock_set_param.side_effect = [True, True, True]
        mock_get_param.side_effect = [0, 0]

        pose = Pose()

        self.assertTrue(ros_server.add_element("key_1", pose))

        self.assertEqual(mock_has_param.call_count, 1)
        self.assertEqual(mock_set_param.call_count, 3)
        self.assertEqual(mock_get_param.call_count, 3)

        element = {'msg_type': pose._type,
                   'msg_value': message_converter.convert_ros_message_to_dictionary(pose),
                   'uuid': 123456789}
        mock_get_param.side_effect = [1, "key_1", element]

        success, info = ros_server.get_element("key_1")

        self.assertTrue(success)
        self.assertEqual(info[0], Pose())

    @mock.patch('rosplan_pytools.controller.scene_database.rospy.has_param')
    @mock.patch('rosplan_pytools.controller.scene_database.rospy.get_param')
    @mock.patch('rosplan_pytools.controller.scene_database.rospy.set_param')
    def test_add_new_element_in_non_empty_db(self, mock_set_param, mock_get_param, mock_has_param):

        mock_has_param.side_effect = [False]
        mock_set_param.side_effect = [True]
        mock_get_param.side_effect = [0]

        ros_server = sdb._RosServerConnection()

        self.assertEqual(mock_has_param.call_count, 1)
        self.assertEqual(mock_set_param.call_count, 1)
        self.assertEqual(ros_server.num_elements(), 0)

        mock_has_param.side_effect = [False]
        mock_set_param.side_effect = [True, True, True]
        mock_get_param.side_effect = [1, "other_key", 1]

        self.assertTrue(ros_server.add_element("key_1", Pose()))

        self.assertEqual(mock_has_param.call_count, 1)
        self.assertEqual(mock_set_param.call_count, 3)
        self.assertEqual(mock_get_param.call_count, 4)

    @mock.patch('rosplan_pytools.controller.scene_database.rospy.has_param')
    @mock.patch('rosplan_pytools.controller.scene_database.rospy.get_param')
    @mock.patch('rosplan_pytools.controller.scene_database.rospy.set_param')
    def test_add_existing_element(self, mock_set_param, mock_get_param, mock_has_param):

        mock_has_param.side_effect = [False]
        mock_set_param.side_effect = [True]
        mock_get_param.side_effect = [0]

        ros_server = sdb._RosServerConnection()

        self.assertEqual(mock_has_param.call_count, 1)
        self.assertEqual(mock_set_param.call_count, 1)
        self.assertEqual(ros_server.num_elements(), 0)

        mock_has_param.side_effect = [False]
        mock_set_param.side_effect = [True, True, True]
        mock_get_param.side_effect = [1, "key_1"]

        self.assertFalse(ros_server.add_element("key_1", Pose()))

        self.assertEqual(mock_has_param.call_count, 1)
        self.assertEqual(mock_set_param.call_count, 1)
        self.assertEqual(mock_get_param.call_count, 3)


class TestRosServerConnectionWithRealRosServer(unittest.TestCase):

    def test(self):

        ros_server = sdb._RosServerConnection()
        ros_server.reset()

        self.assertEqual(ros_server.num_elements(), 0)

        self.assertTrue(ros_server.add_element("key_1", Pose()))
        self.assertEqual(ros_server.num_elements(), 1)

        self.assertTrue(ros_server.add_element("key_2", Pose()))
        self.assertEqual(ros_server.num_elements(), 2)

        self.assertTrue(ros_server.add_element("key_3", Empty()))
        self.assertEqual(ros_server.num_elements(), 3)

        success, element = ros_server.get_element("key_1")
        self.assertTrue(success)
        success, element = ros_server.get_element("key_2")
        self.assertTrue(success)
        success, element = ros_server.get_element("key_3")
        self.assertTrue(success)

        self.assertTrue(ros_server.remove_element("key_1"))
        self.assertEqual(ros_server.num_elements(), 2)

        self.assertTrue(ros_server.remove_element("key_2"))
        self.assertEqual(ros_server.num_elements(), 1)

        self.assertTrue(ros_server.remove_element("key_3"))
        self.assertEqual(ros_server.num_elements(), 0)


if __name__ == '__main__':
    unittest.main()
