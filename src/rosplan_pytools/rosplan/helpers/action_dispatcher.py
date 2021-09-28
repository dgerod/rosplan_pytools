from threading import Lock
import rospy
from rosplan_dispatch_msgs.msg import ActionDispatch, ActionFeedback
from rosplan_pytools.rosplan.common.utils import dict_to_keyval


_lock = Lock()
_waiting_response = True
_action_dispatch_publisher = None
_action_feedback_subscriber = None
ACTION_ID = 100


def _action_feedback_callback(message):

    rospy.loginfo(rospy.get_caller_id() + "I heard %s", message.data)

    if (message.action_id == ACTION_ID and
            message.status in ["action achieved", "action failed"]):
        with _lock.acquire():
            global _waiting_response
            _waiting_response = False


def _wait_action_is_finished():

    running = True
    while running:
        rospy.sleep(0.1)
        with _lock.acquire():
            running = _waiting_response


def initialize(prefix=None):
    if prefix is None:
        prefix = "/kcl_rosplan"

    global _action_feedback_subscriber
    _action_feedback_subscriber = rospy.Publisher(prefix + "/action_feedback",
                                                  ActionFeedback,
                                                  _action_feedback_callback)

    global _action_dispatch_publisher
    _action_dispatch_publisher = rospy.Publisher(prefix + "/action_dispatch",
                                                 ActionDispatch,
                                                 queue_size=10)


def send_action(name, wait_completed=False, **kwargs):

    msg = ActionDispatch()
    msg.action_id = ACTION_ID
    msg.name = name
    msg.parameters = dict_to_keyval(kwargs)
    _action_dispatch_publisher.publish(msg)

    if wait_completed:
        _wait_action_is_finished()
