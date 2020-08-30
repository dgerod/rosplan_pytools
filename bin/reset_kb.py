#! /usr/bin/env python
# -*- coding: utf-8 -*-

from rosplan_pytools.rosplan.controller import knowledge_base as kb


def main():

    kb.initialize()
    kb.reset()


if __name__ == "__main__":
    main()
