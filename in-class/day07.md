---
title: "The Particle Filter for Robot Localiation"
toc_sticky: true
toc_data:
  - title: Today
    link: in-class/day07/#today
  - title: For Next Time
    link: in-class/day07/#for-next-time
  - title: The Particle Filter Algorithm
    link: in-class/day07/#the-particle-filter-algorithm
  - title: C++ Option
    link: in-class/day07/#c-option
---
## Today 

We'll be going over the main steps of how to implement a particle filter.  The goal will be to implement the particle filter at a whiteboard, and use this as the basis for your implementation.

## For Next Time

* [Particle Filter Conceptual Overview](../assignments/robot_localization#conceptual-overview-due-9-22)
* [A view of the finish line](../assignments/robot_localization#a-view-of-the-finish-line-and-getting-set-with-rviz)
* Probability basics assignment

## The Particle Filter Algorithm

A lot of folks expressed the value of working out conceptual issues at a whiteboard prior to coding.  In that spirit, I have modified the robot localization assignment a bit to focus on front-loading that conceptual understanding.  We're going to start down that path in class today, and you'll be finishing things up before next class.

There is an assignment due for next class that asks you to submit, individually, your conceptual breakdown of the particle filter.  Before we get to that, I want to go over the basic steps of the particle filter ot make sure we are clear on what they are.  I will draw a few pictures on the whiteboard, but there will still be more for you to unpack on your own.

Before getting into the basic steps, I want to go over the coordinate systems that we are going to be working with.  I'll discuss the ``map``, ``odom``, and ``base_footprint`` coordinate systems.

Once we have that picture, here are the crucial steps you will need to think about (these are also discussed on the assignment page).

1. Given a pose (represented as $$x, y, \theta$$), compute a set of particles.
2. Given two subsequent odometry poses of your robot, how can you update your particles.
3. Given a laser scan, how can you determine the confidence value (weight) assigned to each particle.
4. Given a weighted set of particles, how can you determine robot's pose?
5. Given a weighted set of particles, how do sample a new set?

I think you have most of the ingredients necessary to think about these questions, but I want to fill in the gaps for (3) and (5).  I'll talk a bit about the concept of a likelihood field and also the idea of random sampling.

Once we go over this basic framework, you'll have the rest of class to start working through the conceptual details with those around you.

## C++ Option

If you are interested in doing your robot localiation project in C++, you can put your name in [this Google sheet](https://docs.google.com/spreadsheets/d/1otiTHnTcRF86xMpGXRr7LC6BBAM2rOIRNFW75xpKt58/edit?usp=sharing).  This will help you find a partner, or if you already have one, connect up with other people who are taking this option.
