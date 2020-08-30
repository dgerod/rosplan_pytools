#! /usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
from rosplan_pytools.rosplan.controller import scene_database as sdb


def main():

    sdb.initialize()

    print("\nElements ---")
    elements = sdb.list_elements()
    print(str(elements) + "\n")

    for name in elements:
        print("[" + name + "] -> [" + str(sdb.get_element(name)[1]) + "]")


if __name__ == "__main__":
    main()
