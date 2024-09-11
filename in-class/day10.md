---
title: "Gauss, Likelihood Fields, and the Particle Filter"
toc_sticky: true
toc_data:
  - title: Today
    link: in-class/day10/#today
  - title: For Next Time
    link: in-class/day10/#for-next-time
  - title: Gaussian Distributions
    link: in-class/day10/#gaussian-distributions
  - title: update_particles_with_laser
    link: in-class/day10/#update_particles_with_laser
  - title: Possible Extensions
    link: in-class/day10/#possible-extensions
---

*** UNDER CONSTRUCTION ***

## Today
We'll discuss the concept of Gaussian distributions and how they apply to particle filters.
 
## For Next Time

* Continue to work on your particle filter project.  Note that the project is due 10/14.

## Gaussian Distributions

We'll start off with a discussion of the Gaussian Distribution.  We'll talk about two operations we might like to perform with respect to the distribution.  The first will be sampling from it.  The second will be evaluating the probability density of the Gaussian at a specific point.  I'll point out ways in which this connects to the particle filter.


## ``update_particles_with_laser``

The first step, as we've touched upon previously, is to determine how the endpoints of the laser scan would fall within the map *if the robot were at the point specified by a particular particle*.  You can do this by drawing some pictures and pulling out your trigonometry skills!

Next, you will take this point and compare it to the map using the provided ``get_closest_obstacle`` function.  Once you have this value, you will need to compute some sort of number that indicates the confidence associated with it.  Finally, you will have to combine multiple confidence values into a single particle weight.  Below are some ways to think about this problem.


### Consult Prob. Rob. for more Detail

To be clear, we are suggesting you implement the likelihood field model.  Here are some resources.  The Beam model is also a commonly used approach, but it is more complex to implement and probably won't perform much better than the simpler approach we are using.

Consider consulting [Probabilistic Robotics](https://docs.ufpr.br/~danielsantos/ProbabilisticRobotics.pdf).  Specifically 6.4 contains the information about likelihood fields.

### Combining Multiple Measurements

As discussed previously, the Gaussian probability density function (PDF) gives us a reasonable way to assign a goodness of fit to a particular scan endpoint.  In order to combine multiple scan endpoints, we can consider a few approaches.

* Average the PDF values across the measurements
* Multiply the PDF values across the measurements
* Something in between?

It turns out these different approaches are all used in various forms in within the particle filters in ROS.  For example, there is a really weird way of combining multiple measurements in the [ROS1 AMCL package](https://github.com/ros-planning/navigation/blob/a9bc9c4c35a55390963db1357926ec461fcff24c/amcl/src/amcl/sensors/amcl_laser.cpp#L293). See this [pull request](https://github.com/ros-planning/navigation/pull/462) for some interesting discussion of this method.

In ROS2, it seems they still have the [old method](https://github.com/ros-planning/navigation2/blob/7be609e67c5b8f7e54b3bc2bcd53d41e652c494e/nav2_amcl/src/sensors/laser/likelihood_field_model.cpp#L124) but there is [another method](https://github.com/ros-planning/navigation2/blob/main/nav2_amcl/src/sensors/laser/likelihood_field_model_prob.cpp) that seems more principled (but may perform worse?).


## Possible Extensions

Here are some possible extensions on the particle filter if you'd like to pursue them.  In the open work time we can provide more detail on how these might work.

* I want to solve the robot kidnapping problem (unknown starting location)
* I want to reimplement the parts of the filter that were written for me (interactions with ROS)
* I want to experiment with laser scan likelihood functions
* I want to try to implement ray tracing instead of the likelihood field
* I want to understand in greater detail the connection between Bayes' filter and the particle filter
* I want to make my particle filter more computationally efficient
* I want to experiment with landmark-based likelihood functions
