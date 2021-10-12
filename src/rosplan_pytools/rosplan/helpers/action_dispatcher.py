from threading import Lock
import rospy
from rosplan_dispatch_msgs.msg import ActionDispatch, ActionFeedback
from rosplan_pytools.rosplan.common.utils import dict_to_keyval


_action_dispatch_publisher = None
_action_feedback_subscriber = None
_lock = Lock()
_action_feedback = None


def _action_feedback_callback(message):

    rospy.loginfo(rospy.get_caller_id() + "I heard %s", message)
    print(rospy.get_caller_id() + "I heard %s", message)

    if message.status in ["action achieved", "action failed"]:
        with _lock:
            global _action_feedback
            _action_feedback = message


def _wait_action_is_finished(id_=None):

    running = True
    while running:

        print("sleep")
        rospy.sleep(0.1)

        with _lock:
            global _action_feedback
            action_feedback = _action_feedback
            _action_feedback = None

            print("id ", id_)
            print("action_feedback ", action_feedback)

            if (action_feedback is not None
                    and id_ is None):
                running = False
            elif (action_feedback is not None
                  and action_feedback.action_id == id_):
                running = False

            print("running ", running)


def initialize(prefix=None):
    if prefix is None:
        prefix = "/kcl_rosplan"

    global _action_feedback_subscriber
    _action_feedback_subscriber = rospy.Subscriber(prefix + "/action_feedback",
                                                   ActionFeedback,
                                                   _action_feedback_callback)

    global _action_dispatch_publisher
    _action_dispatch_publisher = rospy.Publisher(prefix + "/action_dispatch",
                                                 ActionDispatch,
                                                 queue_size=10)


def send_action(id_, name, parameters, wait_completed=False):

    msg = ActionDispatch()
    msg.action_id = id_
    msg.name = name
    msg.parameters = dict_to_keyval(parameters)
    
    #rospy.sleep(5.0)
    _action_dispatch_publisher.publish(msg)

    if wait_completed:
        _wait_action_is_finished(id_)
