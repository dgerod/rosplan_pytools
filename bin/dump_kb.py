#! /usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
from rosplan_pytools.rosplan.controller import knowledge_base as kb


def main():

    kb.initialize()

    print("\n")

    print("INSTANCES ---")
    print(str(kb.list_instances()))
    for instance in kb.list_instances():
        print("[" + str(instance) + ", " + str(kb.get_instance_type(instance)) + "]")
    print("\n")

    print("PREDICATES ---")
    for predicate in kb.list_predicates():
        print("[" + str(predicate) + "]")
    print("\n")

    print("GOALS ---")
    for goal in kb.list_goals():
        print("[" + str(goal) + "]")
    print("\n")


if __name__ == "__main__":
    main()
