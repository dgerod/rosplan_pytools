#! /usr/bin/env python
# -*- coding: utf-8 -*-

from rosplan_interface import kb_interface as kbi

def main():
    kbi.init_kb()
    print "Predicates ---"
    for predicate in kbi.list_predicates():
        print "[" + str(predicate) + "]"
    print "\nInstances ---"
    print str(kbi.list_instances(""))
    print "\nGoals ---"
    for goal in kbi.list_goals():
        print "[" + str(goal) + "]"

if __name__ == '__main__':
    main()
