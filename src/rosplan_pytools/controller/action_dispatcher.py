import rospy
from rosplan_dispatch_msgs.msg import ActionDispatch
from rosplan_pytools.common.utils import dict_to_keyval

_action_dispatch_publisher = None


def init(prefix=None):
    if prefix is None:
        prefix = "/kcl_rosplan"

    global _action_dispatch_publisher
    _action_dispatch_publisher = rospy.Publisher(prefix + "/action_dispatch",
                                                 ActionDispatch,
                                                 queue_size=10)


def send_action(name, **kwargs):
    msg = ActionDispatch()
    msg.name = name
    msg.parameters = dict_to_keyval(kwargs)

    _action_dispatch_publisher.publish(msg)
