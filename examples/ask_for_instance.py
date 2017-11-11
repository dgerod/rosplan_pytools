#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Pose
from rosplan_interface import kb_interface as kbi

# For this example the knowledge base of ROSPlan must be running, and two
# waypoints with name p1 and p2 must exist in it.

def main():

    kbi.init_kb()

    p1, t1 = kbi.get_instance('waypoint', 'p1', Pose._type)
    print 'instance:\n', p1

    p2, t2 = kbi.get_instance(None, 'p2', Pose._type)
    print 'instance:\n', p2

if __name__ == '__main__':
    main()
