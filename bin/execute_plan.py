#! /usr/bin/env python
# -*- coding: utf-8 -*-

from rosplan_pytools.rosplan.controller import planning_system as ps


def main():

    ps.initialize()
    ps.execute_plan()


if __name__ == "__main__":
    main()
