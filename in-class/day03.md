---
title: "Writing our first sensor-moto loop"
toc_sticky: true
---

## Today
* Discussion: <a-no-proxy href="https://thehustle.co/when-robots-kill/">robots and liability</a-no-proxy>
* Writing our first sensory-motor loops (in groups)
* Work on the Warmup Project

## For Next Time
* Read <a-no-proxy href="https://smartech.gatech.edu/bitstream/handle/1853/22221/handbock.pdf">this paper</a-no-proxy> for historical context on reactive robotic control strategies
* Work on the 
<a-no-proxy href="https://olin.instructure.com/courses/143/assignments/440">Warmup Project</a-no-proxy> 

## Sensory Motor Loops

> Thinking back to the first day of class, one way to think of the intelligence that drives a robot is as a sensory-motor loop.

<p align="center">
<img alt="A diagram showing a robot sensory motor mapping interacting with an environment" src="day01images/sensorymotorloops.jpg"/>
</p>

Today, we will kick off a week long project to investigate how we can create sensory-motor loops that can realize intelligent behavior on a robot.  Please push yourself to use object-oriented programming techniques as you go through these examples.  Today in class is a great time to debug some of your understanding of these topics if you are having trouble. 

### Creating our First Sensory Motor Loop

Find another person in the class to work with.  In pairs, you will begin to explore the idea of a sensory-motor loop on the Neatos.

In order to get started, create a package for the code that you will be writing today.  As with all ROS packages in your workspace, it must be inside of your ``catkin_ws/src `` folder.  Besides this requirement, you are free to put the package anywhere, however, I recommend that put it in the root directory of your ``comprobo20`` repository.

```bash
$ cd ~/catkin_ws/src/comprobo20
$ catkin_create_pkg in_class_day03 rospy std_msgs geometry_msgs neato_node sensor_msgs
```

The first sensory-motor loop we will create is one in which the robot moves forward at a fixed speed until it senses an obstacle (using the bump sensor) and then stops.  For a rundown of the bump sensors on the Neato, check out the <a-no-proxy href="https://olin.instructure.com/courses/143/assignments/440">Warmup Project</a-no-proxy> page.

Hints:

* You will need some way to share data between the callback functions that process the Neato's sensory data and your main robot program.  The best way to do this is to use a Python class, but if you are not familiar with these (we will get exposure to them soon), you can use global variables.

Call your node something like ``emergency_stop.py``.  Make sure to make it executable and put it inside a scripts directory in your ``in_class_day03`` ROS package.

### Using the Laser Range Finder

The next sensory-motor loop I suggest that you create should be called ``distance_emergency_stop.py``.  This node should be identical to ``emergency_stop.py`` except it should use the laser range finder to detect when an obstacle is within a specified distance and stop if this is the case. It is up to you how you implement this.  You can either use just the measurements in front of the robot, or perhaps use all of the measurements.  You may want to use an all-or-nothing control strategy (also called bang-bang) where you are either going ahead at some fixed speed or you stop completely.  Alternatively, you may use something akin to proportional control where your speed slows proportionally with how close you are to the target distance.  Again, for more detail on using the Neato sensors (including the laser range finder), see the <a-no-proxy href="https://olin.instructure.com/courses/143/assignments/440">Warmup Project</a-no-proxy> page.


### Wall Bumping and Finite-state Control

Sometimes you want your robot to switch between multiple behaviors.  One very simple architecture for this is to use something called finite-state control.  The idea is that your robot can be in one of a finite number of states.  Each state prescribes a different pattern of behavior for the robot.  The robot transitions between states using various rules.

As an example, let's consider a robot that can be in one of three states:

1. moving forward
2. moving backward
3. rotating left

Each state prescribes a very easy to program behavior.  In each case depending on the robot's state we would either drive forward at a fixed velocity, drive backward at a fixed velocity, or rotate left at a fixed velocity.

Let's say that our robot starts in the **moving forward** state.  We can now define a series of rules for transitioning between the states based on the Neato's sensors.

**moving forward**

* if any bump sensor becomes activated, change state to moving backward.
* otherwise, stay in the moving forward state

**moving backward**

* if the obstacle detected in front of the robot exceeds a specified threshold, change state to rotating left.  Also, record the time at which the robot began rotating left.
* otherwise, stay in the moving backward state.

**rotating left**

* if the current time is more than 1 second greater than when the robot started rotating left, change state to moving forward.
* otherwise, stay in the rotating left state.

What would the behavior of this robot be?  Do a quick whiteboard based simulation to probe the robot's behavior in various situations.  What range of behaviors would the robot exhibit.

With your partner, implement a ROS Python node that realizes this behavior.  For the timing related tasks you will want to check out this ROS documentation page.


### Extension (optional)

Check out the <a-no-proxy href="http://wiki.ros.org/smach/Tutorials">tutorials for the smach ROS package</a-no-proxy> Use smach to either rewrite your finite state controller to use smach, or devise a new finite-state controller and implement it using smach.  