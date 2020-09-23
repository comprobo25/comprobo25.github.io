---
title: "Robot Localization"
toc_sticky: true
toc_h_max: 3
---

## Abstract

So far, you have successfully programmed robot (simulators) using reactive control strategies. You have also combined these reactive control strategies together using finite state control. Next, you will build on these skills as you continue to explore the field of mobile robotics. In this project you will tackle a fundamental problem in mobile robotics: robot localization.


## Learning Objectives

* Build fluency with ROS
* Learn about one core problem (robot localization) and one state of the art algorithm (particle filtering)
* Learn strategies for developing and debugging algorithms for robotics
Get a feel for the algorithmic tradeoffs between accuracy and computational efficiency

## Teaming

For this project, you should work with one other student. Since we have an odd number of students in the class, one team of three will be allowed.

## Deliverables

### (TODO: revise this) Code Architecture Plan (due by the start of class on Friday, September 28th)

You should come to class on Friday with a plan for how you will structure your code for this project.  Your plan could be explained in text, figures, charts, or some combination of these.  The plan should include the following content (not necessarily in this structure).
* What classes will you create for your implementation (everyone should have a ParticleFilter class as given in the starter code, but more are probably better)?
* What functions will be implemented in these classes?
* If you will be using the code in the helper classes, how will your code utilize them.
* Define a timeline of how the code will interact with the sensor data coming from the robot and generate update estimates of the robot's location.

It's possible that you will not be able to nail this down in complete detail by this deadline, but you should at least have thought about these issues and made your best attempt.  If you feel like there are still major parts you don't understand, please add appropriate questions to your writeup so we can discuss them in class.


### (TODO revise deadlines) In-class Presentation / Demo (due by the start of class on Tuesday, October 9th)

Each team will spend about 5 minutes presenting what they did for this project. Since everyone's doing the same project, there's no need to provide any context as to what the particle filter is or how it works.  I'd like each team to talk about what they did that adds to the overall knowledge of the class.  Examples of this would be non-trivial design decisions you made (and why you made them), development processes that worked particularly well, code architecture, etc.  In addition, you should show a demo of your system in action.

This deliverable be assessed in a binary fashion (did you do the above or not).


### (TODO revise deadlines) Your code and bag files (due by the start of class on Tuesday, October 9th)

Your code should be forked from [this repo](https://github.com/comprobo20/robot_localization).  Please push your code to your fork in order to turn it in.

You must include a couple of bag files of your code in action.  Place the bag files in a subdirectory of your ROS package called "bags".  In this folder, create a README file that explains each of the bag files (how they were collected, what you take from the results, etc.).

### Writeup (due by the start of class on Tuesday, October 9th)

In your ROS package create a ``README.md`` file to hold your project writeup.  Your writeup should touch on the following topics:

> TODO: revise based on what we did for the warmup project

* What was the goal of your project?
* How did you solve the problem? (Note: this doesn't have to be super-detailed, you should try to explain what you did at a high-level so that others in the class could reasonably understand what you did).
* Describe a design decision you had to make when working on your project and what you ultimately did (and why)? These design decisions could be particular choices for how you implemented some part of an algorithm or perhaps a decision regarding which of two external packages to use in your project.
* What if any challenges did you face along the way?
* What would you do to improve your project if you had more time?
* Did you learn any interesting lessons for future robotic programming projects? These could relate to working on robotics projects in teams, working on more open-ended (and longer term) problems, or any other relevant topic.


## Robot Localization and the Particle Filter

For this project you will be programming your robot to answer a question that many of us have pondered ourselves over the course of our lifetimes: "Where am I?". The answer to this question is of fundamental importance to a number of applications in mobile robotics such as mapping and path planning.

For this project, we will assume access to a map of the robot's environment. This map could either be obtained using a mapping algorithm (such as SLAM), derived from building blueprints, or even hand-drawn. Given that we have a map of our environment, we can pose the question of "Where am I?" probabilistically. That is, at each instant in time, t, we will construct a probability distribution over the robot's pose within the map, denoted as $$x_t$$, given the robot's sensory inputs, denoted as $$z_t$$ (in this case Lidar measurements), and the motor commands sent to the robot at time t, $$u_t$$.

The particle filter involves the following steps

1. Initialize a set of particles via random sampling
2. Update the particles using data from odometry
3. Reweight the particles based on their compatibility with the laser scan
4. Resample with replacement a new set of particles with probability proportional to their weights.

## Starter Code

### Refresh your ``comprobo20`` repository

In a terminal, Go to your fork of the ``comprobo20`` repository and run the following command.

```bash
$ git pull upstream master
```

### Getting the Robot Localizer Starter Code

The starter code will be in a package called ``robot_localizer`` inside of your fork of the [``robot_localization`` Github repo](https://github.com/comprobo20/robot_localization).  Before you can run the starter code, you must install scikit learn.

> TODO: test all of this on melodic

```bash
$ pip3 install sklearn sklearn
```

You will need some additional ROS packages that we haven't used thus far.

> Note: if you are on Melodic, you would want replace ``noetic`` with ``melodic`` in the commands below.

```bash
$ sudo apt install ros-noetic-map-server ros-noetic-amcl ros-noetic-openslam-gmapping # might be needed for melodic ros-melodic-pointcloud-to-laserscan
```

## Gmapping on Noetic

Gmapping has not been released as a pre-built binary for Noetic yet.  If you are running Noetic, to get around this run the following commands.

> TODO: we should be able to get around doing this on Melodic

```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/ros-perception/slam_gmapping
$ cd ..
$ catkin_make
```


## A View of the Finish Line and Getting Set with RViz

> TODO: make this required between Monday and Wednesday 

Before diving into this project, it helps to have a sense of how a successful implementation of the particle filter functions.  You will be testing ROS's built-in particle filter on a bag file and map that I collected last year during CompRobo.  To get started, run the following command.  Note: before starting this, start roscore in a separate terminal.

```bash
$ roslaunch robot_localizer test_bagfile.launch map_name:=ac109_1 use_builtin:=true
```
If all went well you will streaming text that says

```bash
Hit space to toggle paused, or 's' to step.
[PAUSED ]  Bag Time: 1486503133.050752   Duration: 0.000000 / 124.930935
[PAUSED ]  Bag Time: 1486503133.050752   Duration: 0.000000 / 124.930935
[PAUSED ]  Bag Time: 1486503133.050752   Duration: 0.000000 / 124.930935
[PAUSED ]  Bag Time: 1486503133.050752   Duration: 0.000000 / 124.930935
[PAUSED ]  Bag Time: 1486503133.050752   Duration: 0.000000 / 124.930935
```

You'll likely recognize this as the output of ROS bag.  To start the bag file you would hit the space bar, however, first we'll get thing setup in rviz.

### Getting Set with RViz

> TODO: this happens by default now with the launch file, but it might be worth doing anyway for learning purposes.

In rviz, add the displays for the following topics (use the "By Topic" tab after clicking on the add button).

* ``/particlecloud``
* ``/map_pose``
* ``/map``

Also, change the fixed frame to ``map``.

Using the ``2D Pose Estimate Widget`` (it should be at the top of the rviz bar), click and drag an estimate of for the robot's pose in the map (don't worry about where it is).

> TODO: add an animated gif for this.

### Localizing a Robot

Return to the terminal where the rosbag is playing and click space bar.  Return to rviz.  You should see a cloud of particle in the map that move around with the motion of the robot.  If you want the particle filter to work well, you can update the 2D pose estimate based on where you think the robot is based on the laser scan.  If all goes well, you'll see the robot moving around in the map and the cloud of particles condensing to the true pose of the robot.

## Testing the Particle Filter with the Robot Simulator

In order to code the particle filter, you will need to create a map of the environment you'll be testing in (TODO: suggest a pre-built ``.world`` file).

To make a map, first connect to the robot simualtor (TODO: add instructions for new world file) in the usual way.  Next, run:

```bash
$ roslaunch neato_2dnav gmapping_demo.launch
```

This will start the SLAM code and launch rviz with the appropriate settings.

Pilot the robot around for a while.  The easiest way to do that is to use the following command.

```bash
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

Once you are satisfied with your map, save it by running the following command.

```bash
$ rosrun map_server map_saver -f ~/mymap
```

This will save your map to your home directory as two files (``mymap.pgm`` and ``mymap.yaml``).

You are now done!  You should hit control-c in the terminal window in which you launched ``gmapping_demo.launch``.

In order to test out your map with the built-in particle filter you can run.

```bash
$ roslaunch robot_localizer test_live.launch map_file:=/home/pruvolo/mymap.yaml use_builtin:=true
```

When you start to implement your own particle filter, you can run it using.

```bash
$ roslaunch robot_localizer test_live.launch map_file:=/home/pruvolo/mymap.yaml
```

> Note: You need to put a full path to the map file (relative paths or using the "~" for your home directory will not work)

Follow the same directions for rviz that are detailed above (TODO: add an anchor link) to visualize the state of the filter and set an initial guess as to the pose of the robot.

Next, drive the robot around. If you had a good initial guess, hopefully the robot will be able to localize itself in the environment.

## Validation with Bag Files
We have included some bag files in the repository that you can use to work on the project.  The bag files included are in the following locations.

```bash
robot_localizer/bags/ac109_1.bag
robot_localizer/bags/ac109_2.bag
robot_localizer/bags/ac109_3.bag
robot_localizer/bags/ac109_4.bag
```

Each has an corresponding map file.
```bash
robot_localizer/maps/ac109_1.yaml
robot_localizer/maps/ac109_2.yaml
robot_localizer/maps/ac109_3.yaml
robot_localizer/maps/ac109_4.yaml
```

In order to see what these bags contain, use the ``rosbag info`` command (here we are running this from the ``robot_localization`` directory).

```bash
$ rosbag info robot_localizer/bags/ac109_1.bag
path:        robot_localizer/bags/ac109_1.bag
version:     2.0
duration:    2:04s (124s)
start:       Feb 07 2017 16:32:13.05 (1486503133.05)
end:         Feb 07 2017 16:34:17.98 (1486503257.98)
size:        32.7 MB
messages:    19365
compression: none [24/24 chunks]
types:       geometry_msgs/PoseStamped [d3812c3cbc69362b77dc0b19b345f8f5]
             geometry_msgs/Twist       [9f195f881246fdfa2798d1d3eebca84a]
             nav_msgs/OccupancyGrid    [3381f2d731d4076ec5c71b0759edbe4e]
             nav_msgs/Odometry         [cd5e73d190d741a2f92e81eda573aca7]
             neato_node/Accel          [207a3851a50869ae8ce637885057d51b]
             neato_node/Bump           [459d87767ce0f2ebdc162046e9ad2c13]
             rosgraph_msgs/Log         [acffd30cd6b6de30f120938c17c593fb]
             sensor_msgs/LaserScan     [90c7ef2dc6895d81024acba2ac42f369]
             sensor_msgs/PointCloud    [d8e9c3f5afbdd8a130fd1d2763945fca]
             tf2_msgs/TFMessage        [94810edda583a504dfda3829e70d7eec]
topics:      accel                   2504 msgs    : neato_node/Accel         
             bump                    2502 msgs    : neato_node/Bump          
             cmd_vel                   54 msgs    : geometry_msgs/Twist      
             map_pose                 801 msgs    : geometry_msgs/PoseStamped
             map_pose_continuous     2478 msgs    : geometry_msgs/PoseStamped
             odom                    2499 msgs    : nav_msgs/Odometry        
             projected_stable_scan    895 msgs    : sensor_msgs/PointCloud   
             rosout                    23 msgs    : rosgraph_msgs/Log        
             rosout_agg                11 msgs    : rosgraph_msgs/Log        
             scan                     895 msgs    : sensor_msgs/LaserScan    
             stable_scan              895 msgs    : sensor_msgs/LaserScan    
             tf                      5785 msgs    : tf2_msgs/TFMessage       
             transformed_map           23 msgs    : nav_msgs/OccupancyGrid
```

This bag file has all of the topics you are accustomed to seeing with the addition of two new topics: ``map_pose`` and ``map_pose_continuous``.  These topics each encode the position of the robot in the map frame as determined from  ceiling mounted April tag markers (where the map is defined by the map stored in the maps subdirectory).  The two topics are slightly different in that the ``map_pose`` topic is only published when a marker can be seen by the camera, and the ``map_pose_continuous`` topic is updated with the odometry from the robot even when the robot can't seen one of the markers.  For the purposes of the bag files we have provided, you can use ``map_pose`` for your validation.  ``map_pose_continuous`` may come in handy for working with maps where the robot moves out of range of the markers.

In order to test your code with the bag file, you will want to use a different launch file that will automatically start your particle filter, load the map, and play the bag file.  These instructions should be identical to what you did in the "A View of the Finish Line and Getting Set with RViz" (TODO: add anchor link) section.

```bash
$ roslaunch robot_localizer test_bagfile.launch map_name:=ac109_1
```

As the bag plays you will see the robot move around in the map.  If all is well, the icon of the robot should line up with the red arrow.  The difference between the two tells you the magnitude of the error of the particle filter.  This can, in theory, be used to automatically tune a particle filter (although doing this might be out of the scope of this project).

## Project Hints

* Use the provided rosbags to debug your code (or, for practice, make your own ROS bags) 
* Visualize as many intermediate computations as possible
* Start with simple cases, e.g. start with just one particle, make sure you can visualize the particle in rviz 
* Implement a simple laser scan likelihood function. Test it using just a subset of the Lidar measurements. Make sure that the results conform to what you expect (the simulator or rosbag will be helpful here).
* Improve your laser scan likelihood to maximize performance. Again, rosbag and visualization will be instrumental here.
