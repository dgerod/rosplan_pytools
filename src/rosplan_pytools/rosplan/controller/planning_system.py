"""
  ROSPlan Planning System interface
  A simple way to set, poll, and interrupt plans.
"""

import rospy
from time import sleep
from std_msgs.msg import String

cmd_pub = None
status = None

# Quick reference to avoid looking at ROSPlan source.
#   STATUSES = ["Ready", "Planning", "Dispatching", "Paused"]
#   COMMANDS = ["plan", "pause", "resume", "cancel"]


def initialize():
    global cmd_pub
    cmd_pub = rospy.Publisher("/kcl_rosplan/planning_commands", String)
    rospy.Subscriber("kcl_rosplan/system_state", String, set_dispatch_status)


def is_done():
    return get_dispatch_status() == "Ready"


def get_dispatch_status():
    return status


def set_dispatch_status(msg):
    global status
    status = msg.data


def plan_and_wait():
    # They also have a service that does this, but I have no idea
    #   how they react to pauses and cancels.
    plan()
    # wait for previous item to finish, if there
    if get_dispatch_status() != "Ready":
        while get_dispatch_status() != "Ready":
            sleep(.1)
    # wait for this to start
    while get_dispatch_status() == "Ready":
        sleep(.1)
    # wait for item to finish
    while get_dispatch_status() != "Ready":
        sleep(.1)


def plan():
    cmd_pub.publish(String("plan"))


def pause():
    cmd_pub.publish(String("pause"))


def cancel():
    cmd_pub.publish(String("cancel"))


def resume():
    cmd_pub.publish(String("resume"))
