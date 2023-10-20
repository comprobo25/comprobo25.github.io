---
title: "Keypoints and Descriptors"
toc_sticky: true
---

## Today

* Keypoints, Descriptors, and Keypoint Matching 
* Some project work time

## For Next Time

* Keep on working on the computer vision project
* Readings for our class discussion Tuesday on [Ethical Issues in Computer Vision](https://olin.instructure.com/courses/592/assignments/10886).

## Keypoints, Descriptors, and Keypoint Matching

Use this link access the [slides for today on keypoints and descriptors](https://docs.google.com/presentation/d/1s0RBcPkNVilFvJ-kNjx9kvbEtTkonfxrG7mqxaxuzok/edit?usp=sharing).

We have included three scripts to help you get a better sense of how these things work in practice.  Try running the following demos (make sure you have pulled from upstream first).

### Getting the Code

The code for today is located in the [``class_activities_and_resources`` repository](https://github.com/comprobo22/class_activities_and_resources).  You can find it in the subdirectory ``keypoints_and_descriptors``.  Since this directory is not a ROS package, you do *NOT* need to worry about running ``colcon build``.

### Keypoint-based Tracking

You can use keypoints to track an object as it moves around in an image.  Go to the ``keypoints_and_desciptors`` directory and run the following command.

```bash
$ ./track_meanshift_keypoints.py
```

You should see an image from your computer's webcam appear on the screen.  Click the image once to pause it, a second time to mark the upper-left corner of the tracking image, and a third and final time to mark the lower-right corner of the tracking image.

At this point you should be able to move the object around and the program will attempt to track its position.


### Matching Keypoints

You can use keypoints to match corresponding points in two images.  Go to the ``keypoints_and_desciptors`` directory and run the following command.

```bash
$ ./match_keypoints.py
```

First, make sure you understand what the visualization is showing.  Next, characterize how the matches change as you move the sliders around.  Note, that if you want to recompute matches after using the slider bars, you need to click on the main image window.

### Visualizing SIFT Descriptors

You can visualize the SIFT descript, which we used in the previous two demos.  Go to the ``keypoints_and_desciptors`` directory and run the following command.

```
$ ./visualize_sift.py
```

This visualization is showing the SIFT descriptor that we just covered.  The only thing that it doesn't do is rotate the descriptor relative to the dominant orientation.  Draw in the left pane by clicking and dragging, make sure that you understand why the SIFT descriptor changes the way that it does.  Note, that In order to reset the sketch, you need to hit the spacebar.

### Machine Learning-based Tracking

You may want to try out Magic Leap's model for keypoint identifiation and tracking.  They [have a repository](https://github.com/magicleap/SuperGluePretrainedNetwork) that is pretty easy to get going with.
