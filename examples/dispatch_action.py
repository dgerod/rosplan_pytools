#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from rosplan_pytools.rosplan.controller import action_dispatcher as dispatcher


def main():

    try:
        rospy.init_node("pytools_dispatch_action")
        dispatcher.initialize()

        name = "demo1"
        parameters = {"c": "c1", "o": "o1"}
        dispatcher.send_action(name, **parameters)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
