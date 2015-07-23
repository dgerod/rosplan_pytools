# rosplan_interface
An easier way to hook into kcl_rosplan


## How to use
Somewhere in your code, you have to import and initialize rosplan_interface. Do the initializing *after* you have initialized rospy.
```
import rosplan_interface as planner

if __name__=="__main__":
  rospy.init_node('name')
  planner.init()
  rospy.spin()
```

## Hooking in actions

You can use two method for hooking in new actions for rosplan. This should be make in conjunction with your domain.pddl file, as it uses the names of the pddl actions to find your functions. The following are based on the following:

```
(:action talk
    :parameters (?msg - message ?loc - location)
    :precondition (and (robotat ?loc) (not(said ?msg)))
    :effect (and (not(hasreceivedmessage ?msg ?loca)) (said ?msg))
)
```

An extremely important note is that **you have to set postconditions in your code**. If the postconditions are not set by the time your code completes, kcl_rosplan may assume the next action cannot be executed and trigger a replan. Also, due to kcl_rosplan limitations **everything must be lowercase**. 


### Function-Based

This is only good for quick-and-dirty methods, as they do not support pausing or cancelling. Note that the parameter names match the pddl file.

```
@planner.planner_action
def talk(msg, loc):
  say_stuff()
  set_postconditions()
```

`receive_action` can also take arguments if you don't want to use the same name (or it's taken). Also, argument order does not matter, just the names.

```
@planner.planner_action('talk')
def different_fn_name(loc, msg):
  say_stuff()
  set_postconditions()
```

An action can also fail by raising an exception. Rosplan will handle that and trigger a replan
```
@planner.planner_action
def talk(msg, loc):
  stutter_horribly()
  raise OhDearException()
```



### Class-Based

This is the way to make more robust actions, and function-based actions are automatically redefined into a class.

```
class Talker(planner.Action):
  name = "talk"
  
  def start(msg, loc):
    say_stuff()
    set_postconditions()
    
  def cancel():
    shut_up()
    tear_down()
    
  def pause():
    self.interupted = True
    shut_up()
    
  def resume():
    # arguments are additionally stored as `self.arguments`
    self.start(self.arguments)
```

That's it! The relevant code will be called as rosplan dispatches it (so long as you called the initializer in the beginning)

## Manipulating RosPlan

Sometimes you want to tell rosplan what to do. To do that, you first need to add goals, then run the planner. Better examples are coming, but here's the gist.

```
# Using the KB
planner.add_instance('location', 'loc1')
# you can store stuff into the scene database with a third arg
planner.add_instance('message', 'msg1', std_msgs.msg.String('Be sure to drink your ovaltine'))
planner.add_goal('robotat', {'loc': 'loc1'})
planner.add_goal('hasreceivedmessage', {'msg': 'msg1', 'loc': 'loc1'})

# Then, plan and execute!
planner.plan()

# Now, let's try stopping it
time.sleep(2)
planner.cancel()
```
