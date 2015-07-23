# 'noqa' tells the linter to ignore star warnings.
#   If this file ever becomes complex, remove the
#   stars and add explicit imports.
from .action_interface import * # noqa
from .utils import * # noqa
from .kb_interface import * # noqa
from .dispatch_interface import * # noqa
import rospy


def init_rosplan(block=False):
    """
    Initialize all the things!!

    Loads subscribers and services, assuming you want
      to work with all of Rosplan. No detriment if
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
