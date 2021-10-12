# rosplan_pytools
An easier way to hook into [ROSPlan](https://github.com/KCL-Planning/ROSPlan). This repository is a derivative work of [rosplan_interface](https://github.com/yochan-lab/rosplan_interface).

## How to use
Somewhere in your code, you have to import and initialize `rosplan`. Do the initializing *after* you have initialized rospy.
```
import rosplan_pytools

if __name__=="__main__":
  rospy.init_node("name")
  rosplan_pytools.init()
  rospy.spin()
```

## Hooking in actions
You can use two method for hooking in new actions for ROSPlan. This should be make in conjunction with your domain.pddl file, as it uses the names of the pddl actions to find your functions. The following are based on the following:

```
(:action talk
    :parameters (?msg - message ?loc - location)
    :precondition (and (robotat ?loc) (not(said ?msg)))
    :effect (and (not(hasreceivedmessage ?msg ?loca)) (said ?msg))
)
```

An extremely important note is that depending on which approach is used to create an action **you have to set postconditions in your code**. If the postconditions (or effects) are not set by the time your code completes, ROSPlan may assume the next action cannot be executed and trigger a replan. Also, due to ROSPlan limitations **everything must be lowercase**. 


### Function-Based

This is only good for quick-and-dirty methods, as they do not support pausing or cancelling. Note that the parameter names match the pddl file.

```
@planner.planner_simple_action
def talk(msg, loc):
  say_stuff()
  set_effects()
```

`receive_action` can also take arguments if you don't want to use the same name (or it's taken). Also, argument order does not matter, just the names.

```
@planner.planner_simple_action("talk")
def different_fn_name(loc, msg):
  say_stuff()
  set_effects()
```

An action can also fail by raising an exception. ROSPlan will handle that and trigger a replan
```
@planner.planner_simple_action
def talk(msg, loc):
  stutter_horribly()
  raise OhDearException()
```

In case you are using function-based approach you have to set postconditions (or effects) in your code.

### Class-Based
This is the way to make more robust actions, and function-based actions are automatically redefined into a class.

```
class Talker(planner.SimpleAction):
  name = "talk"
  
  def _start(arguments):
    check_preconditions()
    say_stuff()
    set_effects()
```

That's it! The relevant code will be called as ROSPlan dispatches it (so long as you called the initializer in the beginning)

When you use `SimpleAction` class to create an action you have to set postconditions (or effects) in your code. While setting postconditions is not necessary if you use `Action` class, as this class does it internally using `CheckActionAndProcessEffects` class. 

A special class that could receive for multiple actions exists, it is the `ActionSink`. In addition to the arguments, it is receiving the name of the action that is requested.  

```
class Listener(planner.ActionSink):
  name = ["talker_1", "talker_2"]
  
  def _start(self, action_name, arguments):
    say_stuff()
```

## Manipulating ROSPlan

Sometimes you want to tell ROSPlan what to do. To do that, you first need to add goals, then run the planner.

```
import rosplan_pytools
import rosplan_pytools.rosplan.controller.knowledge_base as kb
import rosplan_pytools.rosplan.controller.scene_database as sdb
import rosplan_pytools.rosplan.controller.planning_system as ps
  
rosplan_pytools.init()
  
# Using the KB
kb.add_instance("loc1", "location")
  
# You can store stuff into the scene database with a third arg
kb.add_instance("msg1", "msg_type")
sdb.add_element("msg1", sdb.Element(std_msgs.msg.String("Be sure to drink your ovaltine"), "msg_type"))

kb.add_goal("robot-at", loc="loc1")
kb.add_goal("has-received-message", msg="msg1", loc="loc1")
  
# Then, plan and execute! using PS
ps.plan()
  
# Now, let's try stopping it
time.sleep(2)
ps.cancel()
```
