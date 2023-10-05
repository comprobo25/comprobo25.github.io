---
title: "Discussions on Robotics and Labor and Computing Relative Motion"
toc_sticky: true
toc_data:
  - title: Today
    link: in-class/day09/#today
  - title: Robotics and Labor Discussion
    link: in-class/day09/#robotics-and-labor-discussion
  - title: Particle Filter and Computing Relative Motion
    link: in-class/day09/#particle-filter-and-computing-relative-motion
---
## Today

* Discussion on Robotics and Labor
* Particle Filter and Computing Relative Motion


## Robotics and Labor Discussion

We'll use [these slides to guide our activities](https://docs.google.com/presentation/d/1-dzppoIzlgZeOh65lj3UMI5Pp33ysFJlYWxPnuz5GDE/edit?usp=sharing).

## Particle Filter and Computing Relative Motion

One part of your particle filter will involve [estimating the relative motion of your robot between two points in time as given by the robot's odometry](https://github.com/comprobo23/robot_localization/blob/d424bff3cd1c3ce12f97aa1e346d6b9866394cdc/robot_localization/pf.py#L215).  Suppose you are given the robot's pose at time $$t_1$$ as a homogeneous transformation matrix (a 2x2 rotation matrix and a 2 dimensional translation vector) along with the robot's pose at time $$t_2$$, how might you compute your robot's change in pose between these two time points?  Think carefully about what coordinate system you want to work in and if you have time, provide an equation to compute the change in pose. 


> Note: while probably not needed for dealing with 2D rotation and translation, I have found the ``PyKDL`` library to be a great way to convert between various representations of orientation and transformations (e.g., see [this section of the starter code](https://github.com/comprobo22/robot_localization/blob/47899d1d1745b56adace25fdff1d08a6bf253a8b/robot_localization/helper_functions.py#L121)).

Here's a video walkthrough I made of using the transformation matrix approach to update particle poses.

> Note: I made a mistake in this video where I showed what multiplying the transformation matrix by a vector gives you (the origin offset was incorrectly calculated).  This doesn't affect the substance of the video, but I wanted to make sure to point it out.

<iframe width="560" height="315" src="https://www.youtube.com/embed/x7mRC0Gowe8?si=movGSLBJvod5ad06" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
