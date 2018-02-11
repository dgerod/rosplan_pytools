#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rosplan_interface as rosplan
from rosplan_interface.action_interface import *


class DemoSimpleAction(SimpleAction):
    name = "demo1"

    def start(self, **kwargs):
        print "Demo 1!!!"


class DemoMultipleSimpleActions(SimpleAction):
    name = ["demo2a", "demo2b"]

    def start(self, **kwargs):
        print "Demo 2!!!"


class DemoActionWithEffects(Action):
    name = 'demo3'

    def start(self, **kwargs):
        print "Demo 3!!!"
        return True


@planner_action
def sample():
    print "Sample!!!"

@planner_action("foobar")
def other():
    print "Hello world."

@planner_action("bad")
def bad(param="Bar"):
    print "Oh no, an error. %s" % param
    raise Exception


def main():
    try:
        rospy.init_node("register_action")
        rosplan.start_actions()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
