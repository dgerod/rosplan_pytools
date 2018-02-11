import rospy

from .action_interface import start_actions
from .kb_interface import init_kb
from .plan_interface import init_dispatch


def init_rosplan(block=False):
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
    start_actions()
    init_kb()
    init_dispatch()

    if block:
        rospy.spin()


init = init_rosplan
