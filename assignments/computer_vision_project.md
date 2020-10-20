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


## Project Topic

Your project should be about computer vision and its intersection with mobile robotics.  While it is likely that a substantial component of your project will be about the vision part (and not necessarily its application to mobile robots), you should scope your project so that you have time to deploy your system on the Neatos.

In your project proposal you will be coming up with an implementation plan.  That is, if you are using a particular algorithm to solve a problem, which parts of the algorithm will you implement, and which will you use pre-built implementations for?  Be strategic in these decisions to balance learning about algorithms with programming the Neatos to do something interesting.  As a brief aside about the pedagogy of this course, since as the course goes on we are going to spend less time discussing algorithms in the structured portion of the class, it is important for you to do project-based exploration of algorithms.  To this end, we expect that you will implement some portion of the algorithms you wind up using.

Suggested Algorithm Topics
* Object tracking
* Image segmentation
* Object detection
* Text recognition
* Fiducial tracking
* Visual odometry
* Structure from motion

(Note: see attached files at the end of of this page for some resources)

## Robot Platform

* TODO: Link to simulators page
* TODO: Datasets list

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
Code
* You should turn in your code and writeup via Github.  Please fork your repo from this one.

### Writeup (Due 11-9)

In your ROS package create a ``README.md`` file to hold your project writeup.  Your writeup should touch on the following topics. We expect this writeup to be done in such a way that you are proud to include it as part of your professional portfolio. As such, please make sure to write the report so that it is understandable to an external audience.  Make sure to add pictures to your report, links to Youtube videos, embedded animated Gifs (these can be recorded with the tool ``peek``).


* What was the goal of your project?  Since everyone is doing a different project, you will have to spend some time setting this context.
* How did you solve the problem (i.e., what methods / algorithms did you use and how do they work)?  As above, since not everyone will be familiar with the algorithms you have chosen, you will need to spend some time explaining what you did and how everything works.
* Describe a design decision you had to make when working on your project and what you ultimately did (and why)? These design decisions could be particular choices for how you implemented some part of an algorithm or perhaps a decision regarding which of two external packages to use in your project.
* What if any challenges did you face along the way?  * What would you do to improve your project if you had more time?  * Did you learn any interesting lessons for future robotic programming projects? These could relate to working on robotics projects in teams, working on more open-ended (and longer term) problems, or any other relevant topic.

## Resources and Potential Project Directions

* [Convert ROS image messages to OpenCV using CvBridge](http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython)
* An overview of methods for object tracking (TODO: old link is broken) 
* [Slides from the CompRobo Learning Project 2018 Class Report Out](https://docs.google.com/presentation/d/1ZZBZotRITt42OIpwC-jiFYWu-h27rZ1PzhctPyRYgng/edit?usp=sharing)
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
