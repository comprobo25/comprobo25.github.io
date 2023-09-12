---
title: "Wall Bumping and Finite-State Control"
toc_sticky: true
---

## Today

* Adjustments to warmup project due dates
* Finite-State Control
* Studio Time

## For Next Time
* Populate the shared Google slide deck with your warmup project lessons and results (final writeup and code due Tuesday the 19th).

## Finite-State Control

Today, we'll do a bit of a lighter dive into Finite State Control than I typically do in the course (leaving more time for work on the warmup project).

First, let's go through the basic ideas together.  We'll draw a basic finite state diagram to describe a robot's behavior.  The main components of this diagram are going to be nodes (these are our states) and edges (which tell us which states follow other states).  Nodes are typically labeled with the name of a state and the edges are labeled with a particular outcome the previous state generates to send the system from the starting state to the next state (where the next state is at the end of the arrow).

## Implementation in ROS2

With folks around you, come up with a way to implement a finite-state controller in ROS2.  Your approach can be very simple or quite complex!  Make sure to outline the main functions, classes, and provide some pseudocode demonstrating your approach.

