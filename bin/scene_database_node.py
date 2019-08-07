#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from rosplan_pytools.controller.nodes import scene_database


def main():

    try:
        scene_database.start_node("scene_database")

    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
