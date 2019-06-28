#! /usr/bin/env python
# -*- coding: utf-8 -*-

from rosplan_pytools.controller import scene_database as sdb


def main():

    sdb.initialize()
    sdb.reset()


if __name__ == "__main__":
    main()
