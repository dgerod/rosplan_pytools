#! /usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import rospy
from rosplan_pytools.interfaces.action_interface import *


class DemoSimpleAction(SimpleAction):
    name = "demo1"

    def start(self, **kwargs):
        print("Demo 1!!!")


def main():
    try:
        rospy.init_node("pytools_register_actions")

        REGISTER_ACTIONS_MANUALLY = False
        register_action(DemoSimpleAction.name, DemoSimpleAction)

        initialize_actions(REGISTER_ACTIONS_MANUALLY)
        start_actions()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
