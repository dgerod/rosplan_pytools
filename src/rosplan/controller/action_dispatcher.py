import rospy
from rosplan_dispatch_msgs.msg import ActionDispatch
from rosplan.common.utils import dict_to_keyval

_action_dispatch_publisher = None


def init():
    global _action_dispatch_publisher
    _action_dispatch_publisher = rospy.Publisher('/kcl_rosplan/action_dispatch',
                                                 ActionDispatch,
                                                 queue_size=10)


def send_action(name, **parameters):
    msg = ActionDispatch()
    msg.name = name
    msg.parameters = dict_to_keyval(parameters)

    _action_dispatch_publisher.publish(msg)
