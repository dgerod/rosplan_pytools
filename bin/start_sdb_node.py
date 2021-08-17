#! /usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import rospy
from rosplan_pytools.rosplan.controller.nodes.scene_database import scene_database


DEFAULT_NODE_NAME = "scene_database"


def usage():
    print("USAGE: %s [node_name][database_name]" % os.path.basename(__file__))
    print("       By default 'node_name' is: '%s', and 'database_name' is same if it is not used." % DEFAULT_NODE_NAME)


def main():

    try:

        num_arguments = len(sys.argv) - 1

        if num_arguments == 0:
            node_name = DEFAULT_NODE_NAME
            sdb_name = node_name
        elif num_arguments == 1:
            node_name = sys.argv[1]
            sdb_name = node_name
        elif num_arguments == 2:
            node_name = sys.argv[1]
            sdb_name = sys.argv[2]
        else:
            usage()
            sys.exit(1)

        # TBD-TEMP @dgerod: Temporary solution for ZAPS
        # This is not working as roslaunch is passing other arguments to the node.
        node_name = DEFAULT_NODE_NAME
        sdb_name = DEFAULT_NODE_NAME

        scene_database.start_node(node_name, sdb_name)

    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
