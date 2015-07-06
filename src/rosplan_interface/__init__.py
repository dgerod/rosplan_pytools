from .action_interface import start_actions, Action, planner_action
from .utils import *
from .kb_interface import init_kb, add_instance, get_instance,\
    add_predicate, rm_instance, rm_predicate, add_goal, get_args

def init_rosplan():
    start_actions()
    init_kb()
