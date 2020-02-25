"""
ROSPlan Action interface
Makes it easy to listen for actions and send feedback
"""

import inspect
import rospy
from rosplan_dispatch_msgs.msg import ActionFeedback, ActionDispatch
from rosplan_knowledge_msgs.srv import GetDomainOperatorDetailsService, GetDomainPredicateDetailsService
from rosplan_pytools.rosplan.controller import knowledge_base as kb
from rosplan_pytools.rosplan.common.utils import keyval_to_dict, dict_to_keyval


DEFAULT_DISPATCH_TOPIC_NAME = "kcl_rosplan/action_dispatch"
DEFAULT_FEEDBACK_TOPIC_NAME = "kcl_rosplan/action_feedback"

feedback = None
action_ids = {}
registered_actions = []
func_action = {}


def _list_actions():
    return []


def _list_existing_actions():

    actions = {}

    # Only one PDDL action is using this class. In case action
    # name is not specified, name of the class is used
    for act in SimpleAction.__subclasses__() + Action.__subclasses__():
        actions[act.name or act.__name__] = act

    # This class is able to receive multiple PDDL actions. Action
    # names are specified as a list.
    for act in ActionSink.__subclasses__():
        for name in act.name:
            actions[name] = act

    return actions


def _list_registered_actions():

    actions = {}
    for act in registered_actions:
        actions[act[0]] = act[1]

    return actions


def _action_receiver(msg):

    global action_ids
    actions = _list_actions()

    rospy.loginfo("[RPpt][AIF] a message is received: '%d', '%s'" % (msg.action_id, msg.name))
    rospy.loginfo(actions)

    if msg.name in actions:

        action_name = msg.name
        rospy.loginfo("[RPpt][AIF] start '%s' action" % action_name)

        try:
            action = actions[action_name](msg.action_id, msg.dispatch_time,
                                          feedback,
                                          keyval_to_dict(msg.parameters))
            action_ids[msg.action_id] = action

            if issubclass(action.__class__, ActionSink):
                action.execute(action_name, **keyval_to_dict(msg.parameters))
            else:
                action.execute(**keyval_to_dict(msg.parameters))

        except Exception as e:
            rospy.logwarn("[RPpt][AIF] action '%s' failed." % msg.name, exc_info=1)
            feedback.publish(ActionFeedback(msg.action_id,
                                            "action failed",
                                            dict_to_keyval(None)))
    elif msg.action_id in action_ids:

        operation = msg.name
        rospy.loginfo("[RPpt][AIF] update action '%d', doing '%s'" % (msg.action_id, operation))

        if operation == "cancel_action":
            action_ids[msg.action_id].cancel()
        elif operation == "pause_action":
            action_ids[msg.action_id].pause()
        elif operation == "resume_action":
            action_ids[msg.action_id].resume()

    else:
        rospy.loginfo("[RPpt][AR] no operation")


def register_action(name, action):

    global registered_actions

    if isinstance(name, str):
        registered_actions.append((name, action))
    else:
        for n in name:
            registered_actions.append((n, action))


def initialize_actions(auto_register_actions=True):

    global _list_actions

    if auto_register_actions:
        _list_actions = _list_existing_actions
    else:
        _list_actions = _list_registered_actions

    rospy.loginfo("[RPpt][AIF] available num actions: %d" % len(_list_actions()))


def start_actions(dispatch_topic_name=None,
                  feedback_topic_name=None,
                  is_blocked=False):

    global feedback

    feedback_topic_name = feedback_topic_name or DEFAULT_FEEDBACK_TOPIC_NAME
    feedback = rospy.Publisher(feedback_topic_name, ActionFeedback, queue_size=10)
    dispatch_topic_name = dispatch_topic_name or DEFAULT_DISPATCH_TOPIC_NAME
    rospy.Subscriber(dispatch_topic_name, ActionDispatch, _action_receiver)

    rospy.loginfo("[RPpt][AIF] Started listening for planner actions")
    if is_blocked:
        rospy.spin()


class SimpleAction(object):
    """
    Receives an action from ROSPlan, automatically sending feedback.
    Extend and set the 'name' attribute (not 'self.name') to define the action.
    
    IMPORTANT: You have to check parameters (pre-conditions) and set effects
    (or post-conditions) in your code. If the effects are not set by the time
    your code completes, ROSPlan may assume the next action cannot be executed
    and trigger a re-plan.
    """
    name = ""

    def __init__(self, action_id, dispatch_time, feedback_pub, arguments):
        self.action_id = action_id
        self.dispatch_time = dispatch_time
        self.feedback_pub = feedback_pub
        self.arguments = arguments
        self.status = "Ready"
        self._report_enabled()

    def _feedback(self, status, info=None):
        self.feedback_pub.publish(ActionFeedback(self.action_id,
                                                 status,
                                                 dict_to_keyval(info)))

    def _report_enabled(self):
        self._feedback("action enabled")

    def _report_success(self):
        self._feedback("action achieved")

    def _report_failed(self):
        self._feedback("action failed")

    def _start(self, **kwargs):
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

    def execute(self, **kwargs):
        return self._start(**kwargs)

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
            self._start(**self.arguments)
            self.status = "Resumed"
        else:
            rospy.logwarn("SimpleAction %s [%i] is not paused." %
                          (self.name, self.action_id))


class ActionSink(object):

    name = []

    def __init__(self, action_id, dispatch_time, feedback_pub, arguments):
        self.action_id = action_id
        self.dispatch_time = dispatch_time
        self.feedback_pub = feedback_pub
        self.arguments = arguments

    def _feedback(self, status, info=None):
        self.feedback_pub.publish(ActionFeedback(self.action_id,
                                                 status,
                                                 dict_to_keyval(info)))

    def _report_enabled(self):
        self._feedback("action enabled")

    def _report_success(self):
        self._feedback("action achieved")

    def _report_failed(self):
        self._feedback("action failed")

    def _start(self, action_name, **kwargs):
        rospy.logwarn("There is supposed to be some code for %s.start()" %
                      self.name)
        raise NotImplementedError

    def execute(self, action_name, **kwargs):
        self._report_enabled()
        if self._start(action_name, **kwargs):
            self._report_success()
        else:
            self._report_failed()


class CheckActionAndProcessEffects(object):
    """
    It validate parameters of an action and applies the effects defined in
    the PDDL file.
    
    NOTE: Based on RPActionInterface class of 'ROSPlan/rosplan_planning_system.
    """
    def __init__(self, action_name, prefix="/kcl_rosplan"):
        self.services = {}

        rospy.wait_for_service(prefix + "/get_domain_operator_details")
        self.services["get_domain_operator_details"] = \
            rospy.ServiceProxy(prefix + "/get_domain_operator_details",
                               GetDomainOperatorDetailsService)

        rospy.wait_for_service(prefix + "/get_domain_predicate_details")
        self.services["get_domain_predicate_details"] = \
            rospy.ServiceProxy(prefix + "/get_domain_predicate_details",
                               GetDomainPredicateDetailsService)

        self.op = None
        self.params = None

        self.pddl_action = action_name
        self.predicates = {}
        self.bound_params = {}

        self._prepare_predicates()

    def _fetch_predicates_from_domain(self, action_name):

        #
        # ---

        res = self.services["get_domain_operator_details"](action_name)
        self.op = res.op
        self.params = res.op.formula

        # Collect predicates from operator description
        # ---

        predicate_names = []

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

        # Effects
        for effect in self.op.at_start_add_effects:
            predicate_names.append(effect.name)
        for effect in self.op.at_start_del_effects:
            predicate_names.append(effect.name)
        for effect in self.op.at_end_add_effects:
            predicate_names.append(effect.name)
        for effect in self.op.at_end_del_effects:
            predicate_names.append(effect.name)

        return predicate_names

    def _fetch_details_of_predicates(self, predicate_collection):
        self.predicates = {}
        for predicate_name in predicate_collection:
            if predicate_name not in self.predicates:
                res = self.services["get_domain_predicate_details"](predicate_name)
                self.predicates[predicate_name] = res.predicate

    def _prepare_predicates(self):
        predicate_collection = self._fetch_predicates_from_domain(self.pddl_action)
        self._fetch_details_of_predicates(predicate_collection)

    def validate_parameters(self, arguments):

        rospy.loginfo("[RPpt][CAPE] validate parameters")

        # Only check parameters, pre-conditions are not considered
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

        rospy.loginfo("[RPpt][CAPE] apply effects")

        # Simple start del effects
        for edx, effect in enumerate(self.op.at_start_del_effects):
            effect_name = effect.name
            effect_value = {}
            for pdx in range(len(effect.typed_parameters)):
                key = self.predicates[effect.name].typed_parameters[pdx].key
                value = self.bound_params[effect.typed_parameters[pdx].key]
                effect_value[key] = value
            kb.remove_predicate(effect_name, **effect_value)

        # Simple start add effects
        for edx, effect in enumerate(self.op.at_start_add_effects):
            effect_name = effect.name
            effect_value = {}
            for pdx in range(len(effect.typed_parameters)):
                key = self.predicates[effect.name].typed_parameters[pdx].key
                value = self.bound_params[effect.typed_parameters[pdx].key]
                effect_value[key] = value
            kb.add_predicate(effect_name, **effect_value)

        # Simple end del effects
        for edx, effect in enumerate(self.op.at_end_del_effects):
            effect_name = effect.name
            effect_value = {}
            for pdx in range(len(effect.typed_parameters)):
                key = self.predicates[effect.name].typed_parameters[pdx].key
                value = self.bound_params[effect.typed_parameters[pdx].key]
                effect_value[key] = value
            kb.remove_predicate(effect_name, **effect_value)

        # Simple end add effects
        for edx, effect in enumerate(self.op.at_end_add_effects):
            effect_name = effect.name
            effect_value = {}
            for pdx in range(len(effect.typed_parameters)):
                key = self.predicates[effect.name].typed_parameters[pdx].key
                value = self.bound_params[effect.typed_parameters[pdx].key]
                effect_value[key] = value
            kb.add_predicate(effect_name, **effect_value)


class Action(object):
    """
    Receives an action from ROSPlan, automatically sending feedback.
    Extend and set the 'name' attribute (not 'self.name') to define the action.

    IMPORTANT: You DON'T have to check parameters (pre-conditions) neither set
    effects (or post-conditions) in your code. As this class set effects by
    itself in case the action does not fail.
    """
    name = ""

    def __init__(self, action_id, dispatch_time, feedback_pub, arguments):
        self.action_id = action_id
        self.dispatch_time = dispatch_time
        self.feedback_pub = feedback_pub
        self.arguments = arguments
        self.status = "Ready"
        self._report_enabled()

    def _feedback(self, status, info=None):
        self.feedback_pub.publish(ActionFeedback(self.action_id,
                                                 status,
                                                 dict_to_keyval(info)))

    def _report_enabled(self):
        self._feedback("action enabled")

    def _report_success(self):
        self._feedback("action achieved")

    def _report_failed(self):
        self._feedback("action failed")

    def _start(self, **kwargs):
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

    def execute(self, **kwargs):

        checker = CheckActionAndProcessEffects(self.__class__.name)

        if not checker.validate_parameters(kwargs):
            raise ValueError("Action arguments are incorrect")

        if not self._start(**kwargs):
            return False

        checker.apply_effects()
        return True

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
            self._start(**self.arguments)
            self.status = "Resumed"
        else:
            rospy.logwarn("Action %s [%i] is not paused." %
                          (self.name, self.action_id))


def planner_simple_action(action_name):
    """
    Decorator to convert a function into an simple action.

    action_name -- The PDDL name of the action.

    """

    def decorator(func):

        class FunctionToAction(SimpleAction):
            name = action_name.lower()  # Lowercase due to ROSPlan quirks
            _spec = inspect.getargspec(func)

            def _start(self, duration=0, **kwargs):
                # Check whether function supports a timeout
                if "duration" in self._spec.args or self._spec.keywords:
                    func(duration, **kwargs)
                else:
                    func(**kwargs)

        global func_action
        func_action[action_name.lower()] = FunctionToAction
        # we don't actually need to do anything,
        # just subclass Action, and dodge lazyness
        return func

    # Be smart if not called with parens
    if callable(action_name):
        func = action_name
        action_name = func.func_name
        return decorator(func)
    else:
        return decorator
