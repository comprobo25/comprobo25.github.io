---
title: "Keypoints and Descriptors"
toc_sticky: true
---

## Today

* Robots and society discussion
* Keypoints, Descriptors, and Keypoint Matching 

## For Next Time

* Keep on working on the computer vision project
* Readings for our class discussion Friday on [Ethical Issues in Computer Vision](https://olin.instructure.com/courses/460/assignments/7725).

## Keypoints, Descriptors, and Keypoint Matching

Use this link access the [slides for today on keypoints and descriptors](https://docs.google.com/presentation/d/1gbDIunTkPLSk01Maq3cgX6IXZmVbTU7qcbXtWda3TwM/view).

We have included two scripts to help you get a better sense of how these things work in practice.  Try running the following demos (make sure you have pulled from upstream first).

```bash
rosrun computer_vision_examples match_keypoints.py
```

First, make sure you understand what the visualization is showing.  Next, characterize how the matches change as you move the sliders around.  Note, that if you want to recompute matches after using the slider bars, you need to click on the main image window.

```
rosrun computer_vision_examples visualize_sift.py
```

This visualization is showing the SIFT descriptor that we just covered.  The only thing that it doesn't do is rotate the descriptor relative to the dominant orientation.  Draw in the left pane by clicking and dragging, make sure that you understand why the SIFT descriptor changes the way that it does.  Note, that In order to reset the sketch, you need to hit the spacebar.
