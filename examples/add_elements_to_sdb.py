#! /usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
from rosplan_pytools.controller import scene_database as sdb
from std_msgs.msg import Empty


def main():

    sdb.initialize()
    sdb.reset()

    print(sdb.num_elements())
    print(sdb.list_elements())

    sdb.add_element("element_1", sdb.Element(Empty()))

    print(sdb.num_elements())
    print(sdb.list_elements())

    sdb.reset()


if __name__ == "__main__":
    main()
