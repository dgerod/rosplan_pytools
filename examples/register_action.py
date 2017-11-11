#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rosplan_interface as rosplan
from rosplan_interface import action_interface as planner

class DemoSimpleAction(planner.SimpleAction):
    name = "Demo1"

    def start(self, **kwargs):
        print "DEMO 1!!!"


class DemoActionWithEffects(planner.Action):
    name = 'Demo2'

    def start(self, **kwargs):
        print "DEMO 2!!!"
        return True


def main():
    try:
        rospy.init_node("register_action")
        rosplan.start_actions(True)

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
