#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from rosplan_pytools.controller import action_dispatcher as dispatcher


def main():
    rospy.init_node("pytools_dipatch_action")
    dispatcher.initialize()

    name = "find_object"
    parameters = {"c": "c1", "o": "o1"}
    dispatcher.send_action(name, **parameters)


if __name__ == "__main__":
    main()
