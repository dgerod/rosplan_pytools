#! /usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
from rosplan_pytools.rosplan.controller import knowledge_base as kb


def main():

    kb.initialize()

    print("Predicates ---")
    for predicate in kb.list_predicates():
        print("[" + str(predicate) + "]")

    print("\nInstances ---")
    print(str(kb.list_instances()) + "\n")
    for instance in kb.list_instances():
        print("[" + str(instance) + ", " + str(kb.get_instance_type(instance)) + "]")

    print("\nGoals ---")
    for goal in kb.list_goals():
        print("[" + str(goal) + "]")


if __name__ == "__main__":
    main()
