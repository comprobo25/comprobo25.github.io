---
title: "Robot Localization"
toc_sticky: true
toc_h_max: 3
---

## Abstract

So far, you have successfully programmed robot (simulators) using reactive control strategies. You have also combined these reactive control strategies together using finite state control. Next, you will build on these skills as you continue to explore the field of mobile robotics. In this project you will tackle a fundamental problem in mobile robotics: robot localization.


## Learning Objectives

* Build fluency with ROS
* Learn about one core problem (robot localization) and one important algorithm (particle filtering)
* Learn strategies for developing and debugging algorithms for robotics
* Get a feel for the algorithmic tradeoffs between accuracy and computational efficiency

## Teaming

For this project, you should work with one other student.

## Deliverables

### Implementation Plan (Due 9-27)

> Note: this is still being finalized soon.  At the latest it will be ready by class on Friday.

You should come to class with a plan for how you will implement this project.

* Map out the key steps of the particle filter.  You should be able to clearly describe using a combination of words and diagrams each step of the algorithm.  If there are parts that are still a bit fuzzy, make sure you take note of them.
* Propose a testing and implementation plan for your particle filter.  Your plan should cover the order in which you will implement the key steps of the particle filter and how you would test to see whether each of them is working.
* Decide whether you will build your code on ``pf.py``, ``pf_scaffold.py``, or create your code from scratch.

It's possible that you will not be able to nail this down in complete detail by this deadline, but you should at least have thought about these issues and made your best attempt.  We will provide feedback on your plan soon after you submit it.

### In-class Presentation / Demo (Due 10-12)

Each team will spend about 5 minutes presenting what they did for this project. Since everyone's doing the same project, there's no need to provide any context as to what the particle filter is or how it works.  I'd like each team to talk about what they did that adds to the overall knowledge of the class.  Examples of this would be non-trivial design decisions you made (and why you made them), development processes that worked particularly well, code architecture, etc.  In addition, you should show a demo of your system in action.

This deliverable be assessed in a binary fashion (did you do the above or not).


### Your Code and Bag Files (Due 10-12)

Your code should be forked from <a href="https://github.com/comprobo22/robot_localization">this repo</a>.  Please push your code to your fork in order to turn it in.

You should include a couple of bag files of your code in action.  Place the bag files in a subdirectory of your ROS package called "bags".  In this folder, create a README file that explains each of the bag files (how they were collected, what you take from the results, etc.).

### Writeup (Due 10-12)

In your ROS package create a ``README.md`` file to hold your project writeup.  Your writeup should touch on the following topics. We expect this writeup to be done in such a way that you are proud to include it as part of your professional portfolio. As such, please make sure to write the report so that it is understandable to an external audience.  Make sure to add pictures to your report, links to Youtube videos, embedded animated Gifs (these can be recorded with the tool ``peek``).


* What was the goal of your project?
* How did you solve the problem? (Note: this doesn't have to be super-detailed, you should try to explain what you did at a high-level so that others in the class could reasonably understand what you did).
* Describe a design decision you had to make when working on your project and what you ultimately did (and why)? These design decisions could be particular choices for how you implemented some part of an algorithm or perhaps a decision regarding which of two external packages to use in your project.
* What if any challenges did you face along the way?
* What would you do to improve your project if you had more time?
* Did you learn any interesting lessons for future robotic programming projects? These could relate to working on robotics projects in teams, working on more open-ended (and longer term) problems, or any other relevant topic.


### Sample Writeups

* <a href="https://github.com/anilpatel38/robot_localization/blob/master/robot_localizer/Anil_Cedric_Localiztion_Report.pdf">Anil Patel and Cedric Kim</a>
* <a href="https://github.com/shootingd/robot_localization">Katya Soltan and Charlie Weiss</a>
* <a href="https://github.com/mary-keenan/robot_localization">Mary Keenan</a>
* <a href="https://github.com/zneb97/robot_localization/blob/master/Robot_Localizer_WriteUp.pdf">Nick Steelman and Ben Ziemann</a>

## Robot Localization and the Particle Filter

For this project you will be programming your robot to answer a question that many of us have pondered ourselves over the course of our lifetimes: "Where am I?". The answer to this question is of fundamental importance to a number of applications in mobile robotics such as mapping and path planning.

For this project, we will assume access to a map of the robot's environment. This map could either be obtained using a mapping algorithm (such as SLAM), derived from building blueprints, or even hand-drawn. Given that we have a map of our environment, we can pose the question of "Where am I?" probabilistically. That is, at each instant in time, t, we will construct a probability distribution over the robot's pose within the map, denoted as $$x_t$$, given the robot's sensory inputs, denoted as $$z_t$$ (in this case Lidar measurements), and the motor commands sent to the robot at time t, $$u_t$$.

The particle filter involves the following steps

1. Initialize a set of particles via random sampling
2. Update the particles using data from odometry
3. Reweight the particles based on their compatibility with the laser scan
4. Resample with replacement a new set of particles with probability proportional to their weights.
5. Update your estimate of the robot's pose given the new particles.  Update the ``map`` to ``odom`` transform.

## Starter Code

### Getting the Robot Localizer Starter Code

The starter code will be in a package called ``robot_localization``.  The <a href="https://github.com/comprobo20/robot_localization"><tt>robot_localization</tt> Github repo</a>  you forked is already setup as an appropriate package.

### Installing Supporting Packages

You will need some additional ROS packages that we haven't used thus far.

```bash
$ pip3 install sklearn
$ sudo apt install ros-foxy-nav2-map-server ros-foxy-nav2-amcl ros-foxy-slam-toolbox
```

### Key Contents of Starter Code

#### ``pf.py``

This file has a basic skeleton of your particle filter (you don't have to build on this if you don't want to).  Check the comments in the file for more details.  You'll probably also want to ask questions as you go through it if something doesn't make sense.  If you want more scaffolding, you can build your code on ``pf_scaffold.py`` instead (see below).


#### ``pf_scaffold.py``

> Note: you will want to either use ``pf.py`` or ``pf_scaffold.py`` (not both)

This file has pretty much all of the interactions with ROS handled.  If you use this as your starting point for your code, you will probably want to overwrite ``pf.py`` with this file (so all of the launch files work as expected).  You should choose this option if you want to focus on implementing the particle filter algorithm and you don't care so much about learning the ins-and-outs of how the particle filter interacts with ROS.

#### ``helper_functions.py``

This file has helper functions for dealing with transforms and poses in ROS. Functionality includes converting poses between various formats, computing the ``map`` to ``odom`` transform, and computing angle differences. 

#### ``occupancy_field.py``

This file implements something called an occupancy field (also called a likelihood field).  This structure has the following capability: given a x,y coordinate in the map, it returns the distance to the closest obstacle in the map.  This can be used as a way to compute the closeness of the match the scan data and the map given a hypothesized location of the robot (particle).

You can find an explanation of the likelihood field model in [Pieter Abbeel's slides](https://people.eecs.berkeley.edu/~pabbeel/cs287-fa12/slides/ScanMatching.pdf) (start at page 11).

## A View of the Finish Line and Getting Set with RViz

> TODO: this is not finished yet

Before diving into this project, it helps to have a sense of how a successful implementation of the particle filter functions.  You will be testing ROS's built-in particle filter on a bag file and map that I collected last year during CompRobo.  To get started, run the following command.

> * Note: before starting this, start roscore in a separate terminal.
> * Note: you should not have Gazebo running as you will be using recorded data from a real robot.

```bash
$ roslaunch robot_localizer test_bagfile_no_rviz.launch map_name:=ac109_1 use_builtin:=true
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

In rviz, add the displays for the following topics (use the ``By Topic`` tab after clicking on the add button).

* ``/particlecloud``
* ``/map_pose``
* ``/map``
* ``/scan`` 

You should also add a visualization for ``Robot Model`` by clicking ``Add`` and then looking on the ``By Display Type`` tab.

Also, change the fixed frame to ``map``.

Using the ``2D Pose Estimate Widget`` (it should be at the top of the rviz bar), click and drag an estimate of for the robot's pose in the map (don't worry about where it is).

Here is a video showing the process in action (note: we forgot to add the ``/scan`` topic in the video, but that should be straightforward..

![A walkthrough of running the particle filter on a bag file](../website_graphics/viewofthefinishline.gif)

#### Troubleshooting

If you see some red text fly by right after unpausing the bag file, the built-in particle filter might have crashed.  This crash is due to a buffer overflow problem that has been fixed in the latest version of ROS Noetic.  To get the updated version of the particle filter that fixes this bug, you can run the following command.

```bash
$ sudo apt-get dist-upgrade
```

### Localizing a Robot

Return to the terminal where the rosbag is playing and click space bar.  Return to rviz.  You should see a cloud of particle in the map that move around with the motion of the robot.  If you want the particle filter to work well, you can update the 2D pose estimate based on the arrow shown by the ``map_pose`` topic.  If all goes well, you'll see the robot moving around in the map and the cloud of particles condensing to the true pose of the robot.

## Testing the Particle Filter with the Robot Simulator

In order to code the particle filter, you will need to create a map of the environment you'll be testing with.  You can use any world you'd like (even make one yourself using the instructions in <a data-canvas="https://olin.instructure.com/courses/143/modules/items/1305" href="../How to/run_the_neato_simulator">running the neato simulator</a>).  We have added a world file for a maze environment (original source: <a href="https://github.com/fsuarez6/labrob">https://github.com/fsuarez6/labrob</a>).

To make a map, first connect to the robot simulator.  If you'd like to use the maze world referenced above, run the following command.

```bash
$ roslaunch neato_gazebo neato_maze_world.launch
```

Next, run:

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

> * Note: You need to put a full path to the map file (relative paths or using the "~" for your home directory will not work)
> * Note: the launch file ``test_live.launch`` will take care of starting ``rviz`` for you.

Next, drive the robot around.  If you had a good initial guess, hopefully the robot will be able to localize itself in the environment.

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

In order to test your code with the bag file, you will want to use a different launch file that will automatically start your particle filter, load the map, and play the bag file.  The instructions below provide a different launch file that will automatically start rviz for you.  If you don't want to start rviz automatically, you can use the instructions in the [Getting Set with RViz section](#getting-set-with-rviz).

```bash
$ roslaunch robot_localizer test_bagfile.launch map_name:=ac109_1
```

As the bag plays you will see the robot move around in the map.  If all is well, the icon of the robot should line up with the red arrow.  The difference between the two tells you the magnitude of the error of the particle filter.  This can, in theory, be used to automatically tune a particle filter (although doing this might be out of the scope of this project).

## Project Advice

### Instructor Advice

* Use the provided rosbags to debug your code (or, for practice, make your own ROS bags) 
* Visualize as many intermediate computations as possible
* Start with simple cases, e.g. start with just one particle, make sure you can visualize the particle in rviz 
* Implement a simple laser scan likelihood function. Test it using just a subset of the Lidar measurements. Make sure that the results conform to what you expect (the simulator or rosbag will be helpful here).
* Improve your laser scan likelihood to maximize performance. Again, rosbag and visualization will be instrumental here.

### Previous Student Advice

Here is some advice from the Fall 2018 class that they wrote up after they completed the robot localization project.

* Pixels vs meters on map (know how these relate to each other)
* Know how coordinate frames as they relate to rviz displays
* Know the what the different rviz topics represent
* Probabilistic justification for parameters (and then cube it for good measure)
* High level understanding (videos) were helpful +1 +1
* Visualization in rviz is crucial
* First pass architecture (should be evaluated / given feedback) (note: this is more of a suggestion for the teaching team than you all).
* Maybe should have done a “ugly, rugged thing” would have helped to get an MVP. Maybe a more iterative process rather than strict planning. All the object we created were hard to glue together.
* Take a “second pass” at the architecture after attempting to implement your first one. Take a step back and ask what you would do differently after having grappled with the low-level code for a week.
* Test each step of the particle filter separately, with full rviz visualizations, with single particles. Create a testing plan!
* Creativity in the weighting algorithm was nice since the x/y distance helper function just worked
* Rosbags are wonderful, wonderful things +1000
* Possibly some more scaffold around researching what amcl does and pull from it to test each part of the filter

## Resources

* <a href="https://www.youtube.com/watch?v=aUkBa1zMKv4">Video Explaining Particle Filter without Equations</a>
* <a href="https://www.youtube.com/watch?v=sz7cJuMgKFg">An Example of a Particle Filter that Might Give More Intuition</a>
* <a href="https://www.youtube.com/watch?v=eAqAFSrTGGY">Very Mathy / Theoretical Treatment of Particle Filter</a> (not for the faint of heart, but we can help you through it)
* A good high-level overview with some great visualizations: <a href="https://www.youtube.com/watch?v=NrzmH_yerBU">Autonomous Navigation: Understanding the Particle Filter</a>

