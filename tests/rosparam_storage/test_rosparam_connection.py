#!/usr/bin/env python

import unittest
import mock

from rosplan_pytools.rosparam.common import ros_message_converter
from rosplan_pytools.rosparam.controller.connection import RosParamsConnection

from std_msgs.msg import Empty
from geometry_msgs.msg import Pose


class TestRosServerConnection(unittest.TestCase):

    @mock.patch('rosparam_storage.controller.connection.rospy.has_param')
    @mock.patch('rosparam_storage.controller.connection.rospy.get_param')
    @mock.patch('rosparam_storage.controller.connection.rospy.set_param')
    def test_connect_when_db_does_not_exist(self, mock_set_param, mock_get_param, mock_has_param):

        mock_has_param.side_effect = [False]
        mock_set_param.side_effect = [True]
        mock_get_param.side_effect = [0, 0]

        connection = RosParamsConnection()

        self.assertEqual(mock_has_param.call_count, 1)
        self.assertEqual(mock_set_param.call_count, 1)
        self.assertEqual(connection.num_elements(), 0)

        # mock.assert_called_with(1, 2, value="alpha")

    @mock.patch('rosparam_storage.controller.connection.rospy.has_param')
    @mock.patch('rosparam_storage.controller.connection.rospy.get_param')
    @mock.patch('rosparam_storage.controller.connection.rospy.set_param')
    def test_connect_when_db_exists(self, mock_set_param, mock_get_param, mock_has_param):

        mock_has_param.side_effect = [True]
        mock_set_param.side_effect = [True]
        mock_get_param.side_effect = [0]

        connection = RosParamsConnection()

        self.assertEqual(mock_has_param.call_count, 1)
        self.assertEqual(mock_set_param.call_count, 0)
        self.assertEqual(connection.num_elements(), 0)

    @mock.patch('rosparam_storage.controller.connection.rospy.has_param')
    @mock.patch('rosparam_storage.controller.connection.rospy.get_param')
    @mock.patch('rosparam_storage.controller.connection.rospy.set_param')
    def test_add_new_element_in_empty_db(self, mock_set_param, mock_get_param, mock_has_param):

        mock_has_param.side_effect = [False]
        mock_set_param.side_effect = [True]
        mock_get_param.side_effect = [0]

        connection = RosParamsConnection()

        self.assertEqual(mock_has_param.call_count, 1)
        self.assertEqual(mock_set_param.call_count, 1)
        self.assertEqual(connection.num_elements(), 0)

        mock_has_param.side_effect = [False]
        mock_set_param.side_effect = [True, True, True]
        mock_get_param.side_effect = [0, 0, 1, 1]

        pose = Pose()

        self.assertTrue(connection.add_element("key_1", pose))

        self.assertEqual(mock_has_param.call_count, 1)
        self.assertEqual(mock_set_param.call_count, 3)
        self.assertEqual(mock_get_param.call_count, 4)

        element = {'msg_type': pose._type,
                   'msg_value': ros_message_converter.convert_ros_message_to_dictionary(pose),
                   'metadata': "some information",
                   'uuid': 123456789}
        mock_get_param.side_effect = [1, "key_1", element]

        success, element = connection.get_element("key_1")

        self.assertTrue(success)
        self.assertEqual(Pose(), element[0])
        self.assertEqual("some information", element[1])

    @mock.patch('rosparam_storage.controller.connection.rospy.has_param')
    @mock.patch('rosparam_storage.controller.connection.rospy.get_param')
    @mock.patch('rosparam_storage.controller.connection.rospy.set_param')
    def test_add_new_element_in_non_empty_db(self, mock_set_param, mock_get_param, mock_has_param):

        mock_has_param.side_effect = [False]
        mock_set_param.side_effect = [True]
        mock_get_param.side_effect = [0]

        connection = RosParamsConnection()

        self.assertEqual(mock_has_param.call_count, 1)
        self.assertEqual(mock_set_param.call_count, 1)
        self.assertEqual(connection.num_elements(), 0)

        mock_has_param.side_effect = [False]
        mock_set_param.side_effect = [True, True, True]
        mock_get_param.side_effect = [1, "other_key", 1, 1, 1]

        self.assertTrue(connection.add_element("key_1", Pose()))

        self.assertEqual(mock_has_param.call_count, 1)
        self.assertEqual(mock_set_param.call_count, 3)
        self.assertEqual(mock_get_param.call_count, 5)

    @mock.patch('rosparam_storage.controller.connection.rospy.has_param')
    @mock.patch('rosparam_storage.controller.connection.rospy.get_param')
    @mock.patch('rosparam_storage.controller.connection.rospy.set_param')
    def test_add_existing_element(self, mock_set_param, mock_get_param, mock_has_param):

        mock_has_param.side_effect = [False]
        mock_set_param.side_effect = [True]
        mock_get_param.side_effect = [0]

        connection = RosParamsConnection()

        self.assertEqual(mock_has_param.call_count, 1)
        self.assertEqual(mock_set_param.call_count, 1)
        self.assertEqual(connection.num_elements(), 0)

        mock_has_param.side_effect = [False]
        mock_set_param.side_effect = [True, True, True]
        mock_get_param.side_effect = [1, "key_1"]

        self.assertFalse(connection.add_element("key_1", Pose()))

        self.assertEqual(mock_has_param.call_count, 1)
        self.assertEqual(mock_set_param.call_count, 1)
        self.assertEqual(mock_get_param.call_count, 3)


class TestRosServerConnectionWithRealRosServer(unittest.TestCase):

    def test(self):

        connection = RosParamsConnection()
        connection.reset()

        self.assertEqual(connection.num_elements(), 0)

        self.assertTrue(connection.add_element("key_1", Pose()))
        self.assertEqual(connection.num_elements(), 1)

        self.assertTrue(connection.add_element("key_2", Pose()))
        self.assertEqual(connection.num_elements(), 2)

        self.assertTrue(connection.add_element("key_3", Empty()))
        self.assertEqual(connection.num_elements(), 3)

        success, element = connection.get_element("key_1")
        self.assertTrue(success)
        success, element = connection.get_element("key_2")
        self.assertTrue(success)
        success, element = connection.get_element("key_3")
        self.assertTrue(success)

        self.assertTrue(connection.remove_element("key_1"))
        self.assertEqual(connection.num_elements(), 2)

        self.assertTrue(connection.remove_element("key_2"))
        self.assertEqual(connection.num_elements(), 1)

        self.assertTrue(connection.remove_element("key_3"))
        self.assertEqual(connection.num_elements(), 0)


if __name__ == '__main__':
    unittest.main()
