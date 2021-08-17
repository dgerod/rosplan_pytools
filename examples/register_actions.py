#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from rosplan_pytools.rosplan.interfaces.action_interface import *


class DemoSimpleAction(SimpleAction):
    name = "demo1"

    def _start(self, **kwargs):
        print("Demo 1!!!")


class DemoActionWithEffects(Action):
    name = "demo3"

    def _start(self, **kwargs):
        print("Demo 3!!!")
        return True


class DemoMultipleActions(ActionSink):
    name = ["demo2a", "demo2b"]

    def _start(self, action_name, **kwargs):
        print("Tracer: %s --> %s" % (DemoMultipleActions.name, action_name))
        return True


@planner_simple_action
def sample():
    print("Sample!!!")


@planner_simple_action("foobar")
def other():
    print("Hello world.")


@planner_simple_action("bad")
def bad(param="Bar"):
    print("Oh no, an error. %s" % param)
    raise Exception


def main():
    try:
        rospy.init_node("pytools_register_actions")

        REGISTER_ACTIONS_AUTOMATICALLY = True
        initialize_actions(REGISTER_ACTIONS_AUTOMATICALLY)

        start_actions()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
