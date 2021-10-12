#! /usr/bin/env python
# -*- coding: utf-8 -*-

from collections import OrderedDict

import rospy
from rosplan_pytools.rosplan.controller import knowledge_base as kb


def _dump_kb():

    print("INSTANCES ---")
    print(str(kb.list_instances()))
    for instance in kb.list_instances():
        print("[" + str(instance) + ", " + str(kb.get_instance_type(instance)) + "]")
    print("\n")

    print("PREDICATES ---")
    for predicate in kb.list_predicates():
        print("[" + str(predicate) + "]")
    print("\n")

    print("FUNCTIONS ---")
    for function in kb.list_functions():
        print("[" + str(function) + "]")
    print("\n")

    print("GOALS ---")
    for goal in kb.list_goals():
        print("[" + str(goal) + "]")
    print("\n")


def main():

    rospy.init_node("example_add_elements_to_kb")

    kb.initialize()
    kb.reset()

    print("Empty KB")
    _dump_kb()

    kb.add_instance("r", "robot")
    kb.add_predicate("robot-at", OrderedDict([("p", "home")]))
    kb.add_predicate("free-tool")
    kb.add_function("occupancy", OrderedDict([('b', 'box_1')]), 0.0)

    print("KB with information")
    _dump_kb()

    kb.remove_instance("r", "robot")
    kb.remove_predicate("robot-at", OrderedDict([("p", "home")]))
    kb.remove_predicate("free-tool")
    kb.remove_function("occupancy", OrderedDict([('b', 'box_1')]), 0.0)

    print("Empty")
    _dump_kb()

    kb.reset()


if __name__ == "__main__":
    main()
