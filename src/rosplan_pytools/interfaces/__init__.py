import rospy
from .action_interface import init as init_actions
from .action_interface import start_actions


def init_interfaces(block=False, auto_register_actions=True):
    """
    Initialize all the things!!

    Loads subscribers and services, assuming you want
      to work with all of ROSPlan. No detriment if
      that isn't the case, but there are smaller methods
      for each component to do what you want.

    Since rospy has to be given a node name before
      anything useful can be done, call this after
      `rospy.init_node(...)`.
    """

    init_actions(auto_register_actions)
    start_actions()

    if block:
        rospy.spin()


init = init_interfaces
