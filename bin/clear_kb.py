#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from rosplan_pytools.controller import knowledge_base as kb


def main():
    kb.init()
    kb.clear_all()


if __name__ == "__main__":
    main()
