#! /usr/bin/env python
# -*- coding: utf-8 -*-

from rosplan_pytools.controller import scene_database as sdb


def main():

    sdb.init()

    print "\nElements ---"
    print str(sdb.list_elements()) + '\n'
    for name in sdb.list_elements():
        print "[" + name + "] -> [" + str(sdb.get_element(name)[1]) + "]"


if __name__ == "__main__":
    main()
