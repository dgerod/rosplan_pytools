#! /usr/bin/env python

import rospy
from rosplan_interface import kb_interface as kbi
from geometry_msgs.msg import Pose, Point, Quaternion


def prepare_kb():
    p = Pose(Point(0.25, .0, .0), Quaternion(0, 0, 0, 1))
    kbi.add_instance('p1', "place", p)
    p = Pose(Point(0.50, .0, .0), Quaternion(0, 0, 0, 1))
    kbi.add_instance('p2', 'place', p)
    p = Pose(Point(0.75, .0, .0), Quaternion(0, 0, 0, 1))
    kbi.add_instance('p3', 'place', p)
    p = Pose(Point(1.00, .0, .0), Quaternion(0, 0, 0, 1))
    kbi.add_instance('p4', 'place', p)

def retrieve_instances():
    p1, p1_type = kbi.get_instance('p1', 'place', Pose._type)
    print 'instance:\n', p1
    p2, p2_type = kbi.get_instance('p2', None, Pose._type)
    print 'instance:\n', p2

    kbi._value_types['place'] = Pose._type
    p3, p3_type = kbi.get_instance('p3', 'place')
    print 'instance:\n', p3
    p4, p4_type = kbi.get_instance('p4')
    print 'instance:\n', p4


def main():

    kbi.init_kb()
    kbi.clear_predicates()
    kbi.clear_goals()

    prepare_kb()
    retrieve_instances()


if __name__ == '__main__':
    main()
