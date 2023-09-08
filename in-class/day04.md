---
title: "Debugging, Proportional Control, and ROS Parameters"
toc_sticky: true
---

## Today
* Robot debugging strategies
* Proportional Control and ROS Parameters
* Studio Time

## For Next Time
* Work on the <a-no-proxy href="../assignments/warmup_project">the Warmup Project</a-no-proxy> (due next Friday, the 15th).


## Brainstorming Robot Debugging Strategies

You've probably learned about debugging strategies for software during your time at Olin.  Many of these generic strategies carry over just fine to debugging robotics programs.  Let's take 10 minutes with the people around you to come up with some debugging strategies for writing robotis code.  Once we you talk to folks around you, we will share out to teh class.  As a motivating example, let's consider the part of the warmup projection where you have to create a person follower.

Here are some areas to consider in the debugging / development lifecycle:
1.  How do you ensure your code is correct (implements the strategy you expected)?
2.  How do you test your approach to see if it performs the task effectively (e.g., follows a person)?
3.  How might you tune the parameters of your approach to make it perform as best possible?

## Proportional Control

So far we've programmed robots to choose between a small set of motor commands (move forward, stop, etc.) based on sensor readings.  Today, we will be experimenting with setting the the motor command  proportional to the error between the robot's current position and the desired position.

To get the idea, program the Neato to adjust its position so that it is a specified (target) distance away from the wall immediately in front of it.  The Neato's forward velocity should be proportional to the error between the target distance and its current distance.  Note: it's tricky to get the sign correct, run through a few mental simulations to make sure the robot will move in the right direction.

To get started, create a package somewhere in your ``ros2_ws/src`` directory for your work.  In this example, I'm going to put the package directly in ``ros2_ws/src/class_activities_and_resources`` directory

```bash
$ cd ~/ros2_ws/src/class_activities_and_resources
$ ros2 pkg create in_class_day04 --build-type ament_python --node-name wall_approach --dependencies rclpy std_msgs geometry_msgs sensor_msgs neato2_interfaces
```

This has been confusing in the past, but whenever you add a new ROS package (or a new ROS node), you'll want to build using ``colcon`` and source the ``setup.bash`` file from your workspace.

```bash
$ cd ~/ros2_ws
$ colcon build --symlink-install
$ source ~/ros2_ws/install/setup.bash
```

You may have noticed at this point that ROS requires a certain amount of [boiler-plate code](https://en.wikipedia.org/wiki/Boilerplate_code) to get going.  If you are having trouble with this, or would rather skip ahead to the proportional control part, you can grab some starter code for [``wall_approach.py``](../Sample_code/wall_approach_starter).

A helpful tool for visualizing the results of your program is to use the <a-no-proxy href="https://docs.ros.org/en/foxy/Concepts/About-RQt.html">rqt</a-no-proxy>.  First, start up the GUI:

```bash
$ rqt
```

Next, go to ``plugins -> visualization -> plot``.

Type ``/scan/ranges[0]`` (if that is in fact what you used to calculate forward distance) into the topic field and then hit +.  You can use this link to find [a sample solution to this task](https://github.com/comprobo23/class_activities_and_resources/blob/main/in_class_day04_solutions/in_class_day04_solutions/wall_approach.py).

### Getting Fancy

To make your node more configurable, use (see the [ros param command line tools documentation](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html) for more information) and [code for accessing parameters in Python documentation from The Robotics BackEnd](https://roboticsbackend.com/rclpy-params-tutorial-get-set-ros2-params-with-python/) (I actually like this doc better than the [official one](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html)). For instance, if you follow the documentation you can create a node similar to our [sample solution](https://github.com/comprobo23/class_activities_and_resources/blob/main/in_class_day04_solutions/in_class_day04_solutions/wall_approach_fancy.py) that supports the following customization via the command line.

```bash
$ ros2 run in_class_day04 wall_approach_fancy --ros-args -p target_distance:=1.5 -p Kp:=0.5
```

Here is a demo of our script, ``wall_approach_fancy.py`` that uses ROS parameters as well as the tool ``dynamic_reconfigure`` for easy manipulation of various node parameters.  As linked above, you use the following link to find [``wall_approach_fancy``](https://github.com/comprobo23/class_activities_and_resources/blob/main/in_class_day04_solutions/in_class_day04_solutions/wall_approach_fancy.py).  Note that in order to support ``dynamic_reconfigure`` you have to call ``add_on_set_parameters_callback`` and implement an appropriate callback function (see sample solution for more on this).  Apologies that the demo below is out of date (you should get the main idea though).

![An animated Gif that shows a robot attempting to maintain a particular distance from a wall](day04images/wall_approach_fancy.gif).
