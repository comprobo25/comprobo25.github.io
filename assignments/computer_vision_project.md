---
title: "Computer Vision Project"
toc_sticky: true
toc_h_max: 3
---

## Abstract

So far, you have successfully programmed robot (simulators) using reactive control strategies and taken a deep dive into the robot localization problem. Next, you will be learning about computer vision and the role it plays in robotics.  In contrast to previous project, this time you will have a great deal of freedom to shape your project towards your interests and learning goals.


## Learning Objectives

* Self-directed learning of a new robotics algorithm.
* Implementing a robotics system with minimal scaffolding.

## Teaming

For this project, you should work with one other student. Since we have an odd number of students in the class, one team of three will be allowed.

Please fill out (<a-no-proxy href="https://docs.google.com/spreadsheets/d/13o6yErKLus7AsdG2PzH4Fh0Jno3YeTf1mArE_2uPMQM/edit?usp=sharing">this Google Sheet</a-no-proxy> when you have a project team and Github repo.

## Project Topic

Your project should be about computer vision and its intersection with robotics.  In our simulated robot reality of the pandemic, this means that you should choose a computer vision topic that has some plausible application in robotics.  You will carry out that project either using a robot simulator, a computer vision dataset, or some combination of both.

In your project proposal you will be coming up with an implementation plan.  That is, if you are using a particular algorithm to solve a problem, which parts of the algorithm will you implement, and which will you use pre-built implementations for?  Be strategic in these decisions to balance learning about algorithms with system building (e.g., programming the simualted Neatos (or another robot) to do something interesting).  You have substantial lattitude in shaping your project to focus on the parts you really want to learn (e.g., system design versus basic understanding of algorithms).  That said, we expect that you will do some exploration of algorithms as part of this project (which could could include implementation or perhaps subsantial learning about an algorithm or class of algorithms).

Potential Algorithm Topics
* Object tracking
* Image segmentation
* Object detection
* Text recognition
* Fiducial tracking
* Visual odometry
* Structure from motion

(Note: there are some resources for these topics later in the document)

## Robot Platform

* One option is to continue to work with the Neato simulator in Gazebo, and use the simulated world for your computer vision project. You will probably want to build a more complex world (see instructions for doing that in the how-tos for the <a-no-proxy href="https://olin.instructure.com/courses/143/modules/items/1305"> Neato Simulator </a-no-proxy>. While we've mostly been using simple shapes in previous projects, there are a lot of built-in models that you can use to build a world. 
* We've been working with the Neato simulator, but there are many other simulators out there, which you're welcome to use in this project. You can find some of them here: <a-no-proxy href="https://olin.instructure.com/courses/143/modules/items/1306"> Other Robot Simulators</a-no-proxy>.
* You may want to use an external dataset for your project. Here are some possible starting points. 
    * [Visual Data](https://www.visualdata.io/discovery) has a nice collection of computer vision datasets and projects.
    * If you want a huge (but very cool) dataset for self-driving vehicles, consider usign Waymo's [Open Dataset](https://waymo.com/open/)
    * [Here's another list of datasets specifically for robotics (not all are about computer vision)](https://lionbridge.ai/datasets/17-best-robotics-datasets-for-machine-learning/)
    * If you're interested in machine learning for robot control, you might consider building off some of <a-no-proxy href="https://github.com/comprobo18/robot_learning/network/members">the datasets (and code) from the 2018 Robot Learning project</a-no-proxy> <a-no-proxy href="https://docs.google.com/presentation/d/1ZZBZotRITt42OIpwC-jiFYWu-h27rZ1PzhctPyRYgng/edit">You also might want to check out [the Google Slides presentation summarizing their results</a-no-proxy>.
    * A lof these datasets are big and fancy, but don't be afraid to start with smaller, more classic datasets.  That's a great way to learn without having to deal with the greater complexity that comes with some of this data.

## Deliverables

There are four deliverables for this project.

### Project Proposal (due 10/26)

At a minimum, please include the answers to the following questions.  You should include enough detail for us to be able to give you useful feedback.

* Who is on your team?
* What is the main idea of your project?
* What are your learning goals for this project?
* What algorithms or computer vision areas will you be exploring?
* What components of the algorithm will you implement yourself, which will you use built-in code for?  Why?
* What is your MVP?
* What is a stretch goal?
* What do you view as the biggest risks to you being successful (where success means achieving your learning goals) on this project?
* What might you need from the teaching team for you to be successful on this project?

### In-class Presentation / Demo (11/9)

We'd like each team to spend about 10 minutes presenting what they did for this project. You can structure the presentation in whatever manner you'd like, however, you should try to meet these goals:
* Explain the goal of your project
* At a high-level explain how your system works
* Demonstrate your system in action (either in a video [recommended] or live). If your system doesn't work completely yet, that is fine, try to show at least one component of your system in action.
* This presentation / demo should be very informal. This presentation will be assessed in a purely binary fashion (basically did you do the things above).


### Code (Due 11-9)

* You should turn in your code and writeup via Github.  Please fork your repo from this one.

### Writeup (Due 11-9)

In your ROS package create a ``README.md`` file to hold your project writeup.  Your writeup should touch on the following topics. We expect this writeup to be done in such a way that you are proud to include it as part of your professional portfolio. As such, please make sure to write the report so that it is understandable to an external audience.  Make sure to add pictures to your report, links to Youtube videos, embedded animated Gifs (these can be recorded with the tool ``peek``).


* What was the goal of your project?  Since everyone is doing a different project, you will have to spend some time setting this context.
* How did you solve the problem (i.e., what methods / algorithms did you use and how do they work)?  As above, since not everyone will be familiar with the algorithms you have chosen, you will need to spend some time explaining what you did and how everything works.
* Describe a design decision you had to make when working on your project and what you ultimately did (and why)? These design decisions could be particular choices for how you implemented some part of an algorithm or perhaps a decision regarding which of two external packages to use in your project.
* What if any challenges did you face along the way?
* What would you do to improve your project if you had more time?
* Did you learn any interesting lessons for future robotic programming projects? These could relate to working on robotics projects in teams, working on more open-ended (and longer term) problems, or any other relevant topic.

## Resources and Potential Project Directions

### Resources

* <a-no-proxy>Convert ROS image messages to OpenCV using CvBridge](http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython)
* [Connecting a webcam to ROS and OpenCV](https://automaticaddison.com/working-with-ros-and-opencv-in-ros-noetic/)
* [An overview of methods for object tracking](https://www.crcv.ucf.edu/papers/Object%20Tracking.pdf)
* <a-no-proxy href="https://docs.google.com/presentation/d/1ZZBZotRITt42OIpwC-jiFYWu-h27rZ1PzhctPyRYgng/edit?usp=sharing">Slides from the CompRobo Learning Project 2018 Class Report Out</a-no-proxy>
* [Mastering OpenCV with Practical Computer Vision Projects](https://www.cs.ccu.edu.tw/~damon/photo/,OpenCV/,Mastering_OpenCV.pdf)
* Visual odometry resources ([one example](http://www.cvlibs.net/software/libviso/))
* [Canny edge detection](https://docs.opencv.org/master/da/d22/tutorial_py_canny.html)
* [Contours](https://docs.opencv.org/master/d4/d73/tutorial_py_contours_begin.html)
* [Template matching](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_template_matching/py_template_matching.html)
* [Hough line transform](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_houghlines/py_houghlines.html)
* [Basics of histograms](https://docs.opencv.org/master/d1/db7/tutorial_py_histogram_begins.html) and [histogram equalization](https://docs.opencv.org/3.4/d4/d1b/tutorial_histogram_equalization.html)
* [Basic Numpy tutorials](https://scipy.github.io/old-wiki/pages/Tentative_NumPy_Tutorial)
* [GUI Features in OpenCV](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_gui/py_table_of_contents_gui/py_table_of_contents_gui.html)
* [Basic Operations on Images](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_core/py_basic_ops/py_basic_ops.html#basic-ops)
* [Arithmetic Operations on Images](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_core/py_image_arithmetics/py_image_arithmetics.html#image-arithmetics)
* [Corner Detection](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_feature2d/py_features_harris/py_features_harris.html)
* [Numpy Examples List](https://scipy.github.io/old-wiki/pages/Numpy_Example_List.html)

### Previous Student Projects to Draw From

* [Visual Navigation for Flying Robots](https://vision.in.tum.de/teaching/ss2013/visnav2013) is a course on said topic.  The linked page includes lectures and even some bag files.
* <a-no-proxy href="https://docs.google.com/presentation/d/1ZZBZotRITt42OIpwC-jiFYWu-h27rZ1PzhctPyRYgng/edit">Robot Learning Report out 2018</a-no-proxy>
* Computer Vision Project Writeups from 2017
   * [Self Driving Neato](https://github.com/arpanrau/self_driving_neato/blob/master/WriteUp.md)
   * [Computer Vision Emotion Detection](https://github.com/BrennaManning/computer_vision_2017/blob/master/writeup.pdf)
   * [Visual Localization](https://github.com/HALtheWise/comprobo-fast-localizer/blob/master/final_report.pdf)
   * [Predicting Paths of Tracked Objects](https://github.com/shanek21/cv_motion_prediction)
   * [Neato Keeper](https://github.com/krusellp/neato_keeper/blob/master/Neato_Keeper.pdf)
   * [Lane Follower](https://github.com/kzhang8850/lane_follower)
   * [Meal Recognition](https://github.com/CompRoboMealVision/meal_recognition)
   * [Pac Neato](https://github.com/jsutker/computer_vision_2017/blob/master/pacneato/scripts/Pacneato_Writeup.md)
   * [Neato Augmented Reality Parking](https://github.com/jovanduy/computer_vision_2017)

