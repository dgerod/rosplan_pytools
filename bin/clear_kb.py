#! /usr/bin/env python
# -*- coding: utf-8 -*-

from rosplan_pytools.rosplan.controller import knowledge_base as kb


def main():

    kb.initialize()
    kb.remove_all_instances()
    kb.remove_all_predicates()
    kb.remove_all_goals()


if __name__ == "__main__":
    main()
