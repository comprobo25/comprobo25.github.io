---
title: "Wall Bumping and Finite-State Control"
toc_sticky: true
---

## Today

* Finite-State Control
* Studio Time

## For Next Time
* Finish the <a-no-proxy href="../assignments/warmup_project">the Warmup Project</a-no-proxy> (due next Tuesday, the 20th).

## Finite-State Control

Today, we'll do a bit of a lighter dive into Finite State Control than I typically do in the course (leaving more time for work on the warmup project).

First, let's go through the basic ideas together.  We'll draw a basic finite state diagram to describe a robot's behavior.  The main components of this diagram are going to be nodes (these are our states) and edges (which tell us which states follow other states).  Nodes are typically labeled with the name of a state and the edges are labeled with a particular outcome the previous state generates to send the system from the starting state to the next state (where the next state is at the end of the arrow).

## Implementation in ROS2

There are quite a few ways to implement state machines.  ROS1 had a really nice library called [smach](http://wiki.ros.org/smach), but there doesn't yet seem to be anything so standardized for ROS2.  The good news is you can create your own implementation of a state machine.  Together, let's write some pseudo-code for a finite-state controller.

First, let's think about what the major functions our finite-state controller will have to perform.  Once we have these mapped out, we'll see if we can create an object-oriented design that you can use within ROS.


