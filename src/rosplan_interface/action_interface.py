"""
RosPlan Action Interface

Makes it easy to listen for actions and send feedback
"""

import inspect
import rospy
from rosplan_dispatch_msgs.msg import ActionFeedback, ActionDispatch

from .utils import keyval_to_dict, dict_to_keyval

func_actions = {}
ids = {}
feedback = None


def action_reciever(msg):
    actions = {}
    for act in Action.__subclasses__():
        actions[act.name or act.__name__] = act
    if msg.name in actions:
        try:
            action = actions[msg.name](msg.action_id,
                                       msg.dispatch_time,
                                       feedback,
                                       keyval_to_dict(msg.parameters))
            ids[msg.action_id] = action
            action.start(**keyval_to_dict(msg.parameters))
            action.report_success()
        except Exception as e:
            rospy.logwarn("Action '%s' failed." % msg.name, exc_info=1)
            feedback.publish(ActionFeedback(msg.action_id,
                                            "action failed",
                                            dict_to_keyval(None)))
    elif msg.name == 'cancel_action':
        if msg.action_id in ids:
            ids[msg.action_id].cancel()
    elif msg.name == 'pause_action':
        if msg.action_id in ids:
            ids[msg.action_id].pause()
    elif msg.name == 'resume_action':
        if msg.action_id in ids:
            ids[msg.action_id].resume()


def start_actions(dispatch_topic_name=None, feedback_topic_name=None):
    global feedback
    dispatch_topic_name = dispatch_topic_name or "kcl_rosplan/action_dispatch"
    feedback_topic_name = feedback_topic_name or "kcl_rosplan/action_feedback"
    feedback = rospy.Publisher(feedback_topic_name,
                               ActionFeedback,
                               queue_size=10)

    rospy.Subscriber(dispatch_topic_name,
                     ActionDispatch,
                     action_reciever)
    rospy.loginfo("Started listening for planner actions")


class Action(object):
    """
    Recieves an action from ROSPlan, automatically sending feedback.
    Extend and set the 'name' attribute (not 'self.name') to define the action.
    """
    name = ""

    def __init__(self, action_id, dispatch_time, feedback_pub, arguments):
        self.action_id = action_id
        self.dispatch_time = dispatch_time
        self.feedback_pub = feedback_pub
        self.arguments = arguments
        self.status = "Ready"
        self.report_enabled()

    def feedback(self, status, info=None):
        self.feedback_pub.publish(ActionFeedback(self.action_id,
                                                 status,
                                                 dict_to_keyval(info)))

    def report_enabled(self):
        self.feedback("action enabled")

    def report_success(self):
        self.feedback("action achieved")

    def report_failed(self):
        self.feedback("action failed")

    def start(self, **kwargs):
        """
        Runs the given task. An exception here will report 'fail', and a
          completion will report success.
        Note: You should make sure long-running functions are
                interruptible with cancel(). If cancelled, you
                should generate an exception of some sort (unless you succeed?)
        """

        rospy.logwarn("There is supposed to be some code for %s.start()" %
                      self.name)
        raise NotImplementedError

    def cancel(self):
        rospy.logwarn("Action %s [%i] has no cancel method." %
                      (self.name, self.action_id))
        self.status = "Cancelled"

    def pause(self):
        rospy.logwarn("Action %s [%i] has no pause method. Cancelling." %
                      (self.name, self.action_id))
        self.status = "Paused"
        self.cancel()

    def resume(self):
        rospy.logwarn("Action %s [%i] has no resume method." %
                      (self.name, self.action_id))
        if self.status == "Paused":
            rospy.logwarn("Action %s [%i] is restarting." %
                          (self.name, self.action_id))
            self.start(**self.arguments)
            self.status = "Resumed"
        else:
            rospy.logwarn("Action %s [%i] is not paused." %
                          (self.name, self.action_id))


def planner_action(action_name):
    """
    Decorator to convert a function into
    an action.

    action_name -- The PDDL name of the action.

    """

    def decorator(func):

        class FunctionAction(Action):
            name = action_name.lower()
            # Lowercase due to ROSPlan quirks
            _spec = inspect.getargspec(func)

            def start(self, duration=0, **kwargs):
                # Check whether function supports a timeout
                if 'duration' in self._spec.args or self._spec.keywords:
                    func(duration, **kwargs)
                else:
                    func(**kwargs)
        func_actions[action_name.lower()] = FunctionAction
        # we don't actually need to do anything,
        #  just subclass Action, and dodge lazyness
        return func

    # Be smart if not called with parens
    if callable(action_name):
        func = action_name
        action_name = func.func_name
        return decorator(func)
    else:
        return decorator

if __name__ == "__main__":
    rospy.init_node("action_demo")

    class DemoAction(Action):
        name = "demo"

        def start(self, **kwargs):
            print "DEMO!!!"

    @planner_action
    def sample():
        print "SAMPLE!!!"

    @planner_action("foobar")
    def other():
        print "Hello world."

    @planner_action("bad")
    def bad(param="Bar"):
        print "Oh no, an error. %s" % param
        raise Exception

    start_actions()
    rospy.spin()
