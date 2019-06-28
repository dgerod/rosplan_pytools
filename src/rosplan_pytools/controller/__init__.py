from .knowledge_base import initialize as init_kb
from .scene_database import initialize as init_sdb
from .planning_system import initialize as init_ps


def init_controller():
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
    init_kb()
    init_sdb()
    init_ps()


init = init_controller
