#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from rosplan_pytools.controller import knowledge_base as kb


def main():
    kb.init()
    print "Predicates ---"
    for predicate in kb.list_predicates():
        print "[" + str(predicate) + "]"

    print "\nInstances ---"
    print str(kb.list_instances()) + '\n'
    for instance in kb.list_instances():
        print "[" + str(kb.get_instance(instance).item) + "]"

    print "\nGoals ---"
    for goal in kb.list_goals():
        print "[" + str(goal) + "]"


if __name__ == "__main__":
    main()
