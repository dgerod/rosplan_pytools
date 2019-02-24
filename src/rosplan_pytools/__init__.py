from .controller import init_controller
from .interfaces import init_interfaces


def init(is_blocked=False):
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
    init_controller()
    init_interfaces(is_blocked)

