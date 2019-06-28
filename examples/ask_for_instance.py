#! /usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import rospy
from rosplan_pytools.controller import knowledge_base as kb
from rosplan_pytools.controller import scene_database as sdb
from geometry_msgs.msg import Pose, Point, Quaternion


def prepare_instances():

    p = Pose(Point(0.25, .0, .0), Quaternion(0, 0, 0, 1))
    kb.add_instance("p1", "place")
    sdb.add_element("p1", sdb.Element(p, "place"))

    p = Pose(Point(0.50, .0, .0), Quaternion(0, 0, 0, 1))
    kb.add_instance("p2", "place")
    sdb.add_element("p2", sdb.Element(p, "place"))

    p = Pose(Point(0.75, .0, .0), Quaternion(0, 0, 0, 1))
    kb.add_instance("p3", "place")
    sdb.add_element("p3", sdb.Element(p, "place"))

    p = Pose(Point(1.00, .0, .0), Quaternion(0, 0, 0, 1))
    kb.add_instance("p4", "place")
    sdb.add_element("p4", sdb.Element(p, "place"))


def retrieve_instances():

    success, p1 = sdb.get_element("p1")
    print("instance:\n", p1.value(), p1.type(), p1.category())

    success, p2 = sdb.get_element("p2")
    print("instance:\n", p2.value(), p2.type(), p2.category())

    success, p3 = sdb.get_element("p3")
    print("instance:\n", p3.value(), p3.type(), p3.category())

    success, p4 = sdb.get_element("p4")
    print("instance:\n", p4.value(), p4.type(), p4.category())


def main():

    rospy.init_node("pytools_ask_for_instance")

    kb.initialize()
    kb.reset()

    sdb.initialize()
    sdb.reset()

    prepare_instances()
    retrieve_instances()


if __name__ == "__main__":
    main()
