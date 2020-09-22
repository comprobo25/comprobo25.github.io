---
title: "Proportional Control and ROS Parameters"
toc_sticky: true
---

## Today

* Wall Bumping and Finite-State Control
* Studio Time

## For Next Time
* Finish the <a-no-proxy href="../assignments/warmup_project" data-canvas="https://olin.instructure.com/courses/143/assignments/440">the Warmup Project</a-no-proxy> (due next Monday, the 28th).

## Wall Bumping and Finite-state Control	

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
