"""
ROSPlan Action Interface

Makes it easy to listen for actions and send feedback
"""

import inspect
import rospy
from rosplan_dispatch_msgs.msg import ActionFeedback, ActionDispatch
from .utils import keyval_to_dict, dict_to_keyval

from rosplan_knowledge_msgs.srv import GetDomainOperatorDetailsService, GetDomainPredicateDetailsService
from . import kb_interface as kbi

feedback = None
action_ids = {}
registered_actions = []
func_action = {}

def start_actions(dispatch_topic_name=None,
                  feedback_topic_name=None,
                  block=False):
    global feedback
    dispatch_topic_name = dispatch_topic_name or "kcl_rosplan/action_dispatch"
    feedback_topic_name = feedback_topic_name or "kcl_rosplan/action_feedback"
    feedback = rospy.Publisher(feedback_topic_name,
                               ActionFeedback,
                               queue_size=10)

    rospy.Subscriber(dispatch_topic_name,
                     ActionDispatch,
                     action_receiver)
    rospy.loginfo("Started listening for planner actions")
    if block:
        rospy.spin()


def register_action(name, action):
    global registered_actions
    registered_actions.append((name,action))


def action_receiver(msg):

    global action_ids

    actions = {}
    for act in SimpleAction.__subclasses__():
        actions[act.name or act.__name__] = act
    for act in Action.__subclasses__():
        actions[act.name or act.__name__] = act
    #for act in registered_actions:
    #    actions[act[0]] = act[1]

    if msg.name in actions:
        try:
            action = actions[msg.name](msg.action_id,
                                       msg.dispatch_time,
                                       feedback,
                                       keyval_to_dict(msg.parameters))
            action_ids[msg.action_id] = action

            if action.__class__ == SimpleAction:
                action.start(**keyval_to_dict(msg.parameters))
            else:
                action.execute(**keyval_to_dict(msg.parameters))
            action.report_success()

        except Exception as e:
            rospy.logwarn("action '%s' failed." % msg.name, exc_info=1)
            feedback.publish(ActionFeedback(msg.action_id,
                                            "action failed",
                                            dict_to_keyval(None)))
    elif msg.name == 'cancel_action':
        if msg.action_id in action_ids:
            action_ids[msg.action_id].cancel()
    elif msg.name == 'pause_action':
        if msg.action_id in action_ids:
            action_ids[msg.action_id].pause()
    elif msg.name == 'resume_action':
        if msg.action_id in action_ids:
            action_ids[msg.action_id].resume()


class SimpleAction(object):
    """
    Receives an action from ROSPlan, automatically sending feedback.
    Extend and set the 'name' attribute (not 'self.name') to define the action.
    
    IMPORTANT: You have to set effects (or postconditions) in your code. 
    If the effects are not set by the time your code completes, ROSPlan may 
    assume the next action cannot be executed and trigger a replan.
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

        NOTE: You should make sure long-running functions can be interrupted
        with cancel(). If cancelled, you should generate an exception of some 
        sort (unless you succeed?)
        """

        rospy.logwarn("There is supposed to be some code for %s.start()" %
                      self.name)
        raise NotImplementedError

    def cancel(self):
        rospy.logwarn("SimpleAction %s [%i] has no cancel method." %
                      (self.name, self.action_id))
        self.status = "Cancelled"

    def pause(self):
        rospy.logwarn("SimpleAction %s [%i] has no pause method. Cancelling." %
                      (self.name, self.action_id))
        self.status = "Paused"
        self.cancel()

    def resume(self):
        rospy.logwarn("SimpleAction %s [%i] has no resume method." %
                      (self.name, self.action_id))
        if self.status == "Paused":
            rospy.logwarn("SimpleAction %s [%i] is restarting." %
                          (self.name, self.action_id))
            self.start(**self.arguments)
            self.status = "Resumed"
        else:
            rospy.logwarn("SimpleAction %s [%i] is not paused." %
                          (self.name, self.action_id))


class CheckActionAndProcessEffects(object):
    """
    It checks if the parameters of the action are correct according to PDDL definition,
    and it applies the effects defined in the PDDL file. 
    
    NOTE: Based on RPActionInterface class of 'ROSPlan/rosplan_planning_system.
    """
    def __init__(self, action_name, prefix="/kcl_rosplan"):
        rospy.loginfo("ProcessEffectOfAction::init")

        self.services = {}

        rospy.wait_for_service(prefix + "/get_domain_operator_details")
        self.services['get_domain_operator_details'] = \
            rospy.ServiceProxy(prefix + "/get_domain_operator_details",
                               GetDomainOperatorDetailsService)

        rospy.wait_for_service(prefix + "/get_domain_predicate_details")
        self.services['get_domain_predicate_details'] = \
            rospy.ServiceProxy(prefix + "/get_domain_predicate_details",
                               GetDomainPredicateDetailsService)

        self.op = None
        self.params = None

        self.pddl_action = action_name
        self.predicates = {}
        self.bound_params = {}

    def _fetch_predicates_from_domain(self, action_name):

        #
        # ---

        res = self.services['get_domain_operator_details'](action_name)
        self.op = res.op
        self.params = res.op.formula

        # Collect predicates from operator description
        # ---

        predicate_names = []

        # Effects
        for effect in self.op.at_start_add_effects:
            predicate_names.append(effect.name)
        for effect in self.op.at_start_del_effects:
            predicate_names.append(effect.name)
        for effect in self.op.at_end_add_effects:
            predicate_names.append(effect.name)
        for effect in self.op.at_end_del_effects:
            predicate_names.append(effect.name)

        # Simple conditions
        for condition in self.op.at_start_simple_condition:
            predicate_names.append(condition.name)
        for condition in self.op.over_all_simple_condition:
            predicate_names.append(condition.name)
        for condition in self.op.at_end_simple_condition:
            predicate_names.append(condition.name)

        # Negative conditions
        for condition in self.op.at_start_neg_condition:
            predicate_names.append(condition.name)
        for condition in self.op.over_all_neg_condition:
            predicate_names.append(condition.name)
        for condition in self.op.at_end_neg_condition:
            predicate_names.append(condition.name)

        return predicate_names

    def _fetch_details_of_predicates(self, predicate_collection):
        self.predicates = {}
        for predicate_name in predicate_collection:
            if predicate_name not in self.predicates:
                res = self.services['get_domain_predicate_details'](predicate_name)
                self.predicates[predicate_name] = res.predicate
                print(res.predicate)

    def prepare(self):
        predicate_collection = self._fetch_predicates_from_domain(self.pddl_action)
        self._fetch_details_of_predicates(predicate_collection)

    def check_parameters(self, arguments):

        found = [False] * len(self.params.typed_parameters)

        for pdx, parameter in enumerate(self.params.typed_parameters):
            for adx in range(len(arguments)):
                if parameter.key == arguments.keys()[adx]:
                    self.bound_params[arguments.keys()[adx]] = arguments.values()[adx]
                    found[pdx] = True
                    break

            if not found[pdx]:
                return False

        return True

    def apply_effects(self):

        # Simple start del effects
        for edx, effect in enumerate(self.op.at_start_del_effects):
            effect_name = effect.name
            effect_value = {}
            for pdx in range(len(effect.typed_parameters)):
                key = self.predicates[effect.name].typed_parameters[pdx].key
                value = self.bound_params[effect.typed_parameters[pdx].key]
                effect_value[key] = value
            kbi.rm_predicate(effect_name, **effect_value)

        # Simple start add effects
        for edx, effect in enumerate(self.op.at_start_add_effects):
            effect_name = effect.name
            effect_value = {}
            for pdx in range(len(effect.typed_parameters)):
                key = self.predicates[effect.name].typed_parameters[pdx].key
                value = self.bound_params[effect.typed_parameters[pdx].key]
                effect_value[key] = value
            kbi.add_predicate(effect_name, **effect_value)

        # Simple end del effects
        for edx, effect in enumerate(self.op.at_end_del_effects):
            effect_name = effect.name
            effect_value = {}
            for pdx in range(len(effect.typed_parameters)):
                key = self.predicates[effect.name].typed_parameters[pdx].key
                value = self.bound_params[effect.typed_parameters[pdx].key]
                effect_value[key] = value
            kbi.rm_predicate(effect_name, **effect_value)

        # Simple end add effects
        for edx, effect in enumerate(self.op.at_end_add_effects):
            effect_name = effect.name
            effect_value = {}
            for pdx in range(len(effect.typed_parameters)):
                key = self.predicates[effect.name].typed_parameters[pdx].key
                value = self.bound_params[effect.typed_parameters[pdx].key]
                effect_value[key] = value
            kbi.add_predicate(effect_name, **effect_value)


class Action(object):
    """
    Receives an action from ROSPlan, automatically sending feedback.
    Extend and set the 'name' attribute (not 'self.name') to define the action.

    IMPORTANT: You DON'T have to set effects (or postconditions) in your code. 
    As this class set effects (or postconditions) by itself in case the action
    does not fail.
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

    def execute(self, **kwargs):
        checker = CheckActionAndProcessEffects(self.__class__.name)
        checker.prepare()

        if checker.check_parameters(kwargs) == False:
            raise ValueError("Action arguments are incorrect")
        if self.start(**kwargs) == True:
            checker.apply_effects()

    def start(self, **kwargs):
        """
        Runs the given task. An exception here will report 'fail', and a
        completion will report success.
          
        NOTE: You should make sure long-running functions can be interrupted
        with cancel(). If cancelled, you should generate an exception of some 
        sort (unless you succeed?)
        """

        rospy.logwarn("There is supposed to be some code for %s.execute()" %
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
    Decorator to convert a function into an action.

    action_name -- The PDDL name of the action.

    """

    def decorator(func):

        global func_action

        class FunctionToAction(SimpleAction):
            name = action_name.lower()
            # Lowercase due to ROSPlan quirks
            _spec = inspect.getargspec(func)

            def start(self, duration=0, **kwargs):
                # Check whether function supports a timeout
                if 'duration' in self._spec.args or self._spec.keywords:
                    func(duration, **kwargs)
                else:
                    func(**kwargs)

        func_action[action_name.lower()] = FunctionToAction
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

    class DemoSimpleAction(SimpleAction):
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
