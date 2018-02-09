#! /usr/bin/env python

"""
For this example the knowledge base of ROSPlan must be running, and three
waypoints with name p1, p2 and p3, must exist in it.
"""

import rospy
from rosplan_interface import kb_interface as kbi
from geometry_msgs.msg import Pose, Point, Quaternion

def prepare_kb():
    p = Pose(Point(-0.2, 0, 0), Quaternion(0, 0, 0, 1))
    kbi.add_instance('p1', "place", p)
    p = Pose(Point(-0.5, -0.4, 0), Quaternion(0, 0, 0, 1))
    kbi.add_instance('p2', 'place', p)
    p = Pose(Point(-1.0, -0.4, 0), Quaternion(0, 0, 0, 1))
    kbi.add_instance('p3', 'place', p)


def retrieve_instances():
    p1, p1_type = kbi.get_instance('p1', 'place', Pose._type)
    print 'instance:\n', p1
    p2, p2_type = kbi.get_instance('p2', None, Pose._type)
    print 'instance:\n', p2

    kbi.types = { 'place' : Pose._type}
    p3, p3_type = kbi.get_instance('p3', 'place')
    print 'instance:\n', p3


def main():

    kbi.init_kb()
    kbi.clear_predicates()
    kbi.clear_goals()

    prepare_kb()
    retrieve_instances()


if __name__ == '__main__':
    main()
