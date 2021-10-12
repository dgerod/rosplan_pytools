#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from rosplan_pytools.rosplan.helpers import action_dispatcher as dispatcher


def main():

    try:
        rospy.init_node("pytools_dispatch_action")
        dispatcher.initialize()

        id_ = 0
        name = "find_object"
        parameters = {"c": "bin_2", "o": "gear_4"}
        dispatcher.send_action(id_, name, parameters)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
