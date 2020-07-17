"""
  ROSPlan Planning System interface
  A simple way to set, poll, and interrupt plans.
"""

import rospy
from std_srvs.srv import Empty


DEFAULT_PROBLEM_NODE_NAME = "/rosplan_problem_generator"
DEFAULT_PLANNER_NODE_NAME = "/rosplan_plan_generator"
DEFAULT_PARSER_NODE_NAME = "/rosplan_plan_parser"
DEFAULT_DISPATCHER_NODE_NAME = "/rosplan_plan_dispatcher"

_services = {}


def _wait_until_node_is_ready(service_name):
    rospy.wait_for_service(service_name)


def _initialize_problem_services(prefix):

    global _services

    service_name = prefix + "/problem_generation_server"
    _services["generate_problem"] = \
        rospy.ServiceProxy(service_name, Empty)

    _wait_until_node_is_ready(service_name)


def _initialize_planner_services(prefix):

    global _services

    service_name = prefix + "/planning_server"
    _services["generate_plan"] = \
        rospy.ServiceProxy(service_name, Empty)

    _wait_until_node_is_ready(service_name)


def _initialize_parser_services(prefix):

    global _services

    service_name = prefix + "/parse_plan"
    _services["parse_plan"] = \
        rospy.ServiceProxy(service_name, Empty)

    _wait_until_node_is_ready(service_name)


def _initialize_dispatcher_services(prefix):

    global _services

    service_name = prefix + "/dispatch_plan"
    _services["dispatch_plan"] = \
        rospy.ServiceProxy(service_name, Empty)

    service_name = prefix + "/cancel_plan"
    _services["cancel_dispatch"] = \
        rospy.ServiceProxy(service_name, Empty)

    _wait_until_node_is_ready(prefix + "/cancel_dispatch")


def initialize(problem_node_name=None, planner_node_name=None, parser_node_name=None,
               dispatcher_node_name=None):
    _initialize_problem_services(problem_node_name or DEFAULT_PROBLEM_NODE_NAME)
    _initialize_planner_services(planner_node_name or DEFAULT_PLANNER_NODE_NAME)
    _initialize_parser_services(parser_node_name or DEFAULT_PARSER_NODE_NAME)
    _initialize_dispatcher_services(dispatcher_node_name or DEFAULT_DISPATCHER_NODE_NAME)


def generate_plan():
    _services["generate_problem"]()
    _services["generate_plan"]()


def execute_plan():
    _services["parse_plan"]()
    _services["dispatch_plan"]()


def plan():
    _services["generate_problem"]()
    _services["generate_plan"]()
    _services["parse_plan"]()
    _services["dispatch_plan"]()


def plan_and_wait():
    pass


def cancel():
    _services["cancel_dispatch"]()
