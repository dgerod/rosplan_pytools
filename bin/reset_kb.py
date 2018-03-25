#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from rosplan.controller import knowledge_base as kb


def main():
    kb.init()
    kb.clear_predicates()
    kb.clear_goals()


if __name__ == "__main__":
    main()
