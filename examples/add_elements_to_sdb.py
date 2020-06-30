#! /usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
from rosplan_pytools.rosplan.controller import scene_database as sdb

from std_msgs.msg import String


def main():

    sdb.initialize()
    sdb.reset()

    print("num elements: ", sdb.num_elements())
    print("elements: ", sdb.list_elements())

    raw_input("Add elements. Press Enter to continue...")

    success = sdb.add_element("element_1", sdb.Element(String("value_1"), "metadata_1"))
    success = sdb.add_element("element_2", sdb.Element(String("value_2"), "metadata_2"))
    success = sdb.add_element("element_3", sdb.Element(String("value_3"), "metadata_3"))

    print("num elements: ", sdb.num_elements())
    print("elements: ", sdb.list_elements())

    raw_input("Add existing element. Press Enter to continue...")

    success = sdb.add_element("element_1", sdb.Element(String("value_100"), "metadata_100"))

    print("added: ", success)
    print("num elements: ", sdb.num_elements())
    print("elements: ", sdb.list_elements())

    raw_input("Remove element. Press Enter to continue...")

    sdb.remove_element("element_2")

    print("num elements: ", sdb.num_elements())
    print("elements: ", sdb.list_elements())

    raw_input("Get an element. Press Enter to continue...")

    success, element = sdb.get_element("element_3")
    print("element: ", element)
    print("num elements: ", sdb.num_elements())

    raw_input("Remove all element. Press Enter to continue...")

    print("num elements: ", sdb.num_elements())

    sdb.remove_elements()

    print("num elements: ", sdb.num_elements())


if __name__ == "__main__":
    main()
