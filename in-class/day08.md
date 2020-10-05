## Coordinate Frames and Robot Localization

* Wrap-up on last week's lecture
* Practical considerations for particle filters
* Debrief on implementation plans for particle filter
* Discussion on Legal Issues in Robotics

## For Next Time
* Work on the <a-no-proxy href="https://olin.instructure.com/courses/143/assignments/1325">Robot Localization project</a-no-proxy>.
* Discussion Readings: 
  * <a-no-proxy href="https://obamawhitehouse.archives.gov/blog/2015/05/08/ensuring-students-have-equal-access-stem-courses"> Equal access to STEM</a-no-proxy>
  * <a-no-proxy href="https://medium.com/@furhatrobotics/a-robot-in-every-classroom-furhats-vision-for-education-5b0ca8d56e0e"> Robots in classrooms </a-no-proxy>
  * <a-no-proxy href="https://new.abb.com/news/detail/4431/abb-and-the-economist-launch-automation-readiness-index-global-ranking-for-robotics-and-artificial-intelligence"> Automation globally </a-no-proxy>

## Wrap-up on Particle Filter Theory Lecture

## The Particle Filter and Coordinate Frames

> Note: we'll quickly review the idea of transformation matrices to encode coordinate frames, but if you want to see things in more detail consider watching the video on coordinate frames from [day 2](day02).

When we think of robot localization, there are usually two ways that robots estimate motion.
* **Odometry:** robots use sensors to determine their motion over short windows of time and use the dead-reckoning to estimate their current position and orientation.  Sensors such as wheel encoders (that measure how far wheels turn), accelerometers, and gyroscopes are typical choices for odometry.
  * Pros: very fast to update (typically requires minimal computation), updates can always be performed (these sorts of sensors usually always give usable data).
  * Cons: errors accumulate over time, estimates can be very inaccurate sometimes (e.g., wheel slippage). 
* **Landmarks:** robots can detect landmarks in their environment as a way to determine their position.  Landmark-based motion estimation is able to cancel out errors that have accumulated in the odometry-based motion estimation by referencing the current position to the robot's current position to its position at some other, potentially distant point in time.
  * Pros: Able to cancel out accumulated error in position.
  * Cons: often computationally expensive and there are not always suitable landmarks to use for such motion estimation (i.e., landmarks may only be intermittently available).

When implementing our particle filter we'd like to get the best of both words: the fast and always available updates of odometry and the ability to cancel out accumulated motion estimation error using landmarks.  We can use the concept of coordinate frames in ROS as a way to accomplish this.

### The map->odom->base_link chain

In the warmup project we met the coordinate frames ``odom`` and ``base_link``.  Recall that ``base_link`` is a coordinate system that rides around with the Neato.  Also recall that ``odom`` is a coordinate system that initially starts aligned with ``base_link`` (meaning the robot starts at the origin of ``odom``) and the relationship between ``odom`` and ``base_link`` is constantly updated to encode the robot's estimate of its position relative to its starting location using its odometry. 

In the particle filter project we are introducing an additional coordinate frame called ``map``.  The ``map`` frame is defined in terms of a map that our particle filter code is given.  Just as the transform between ``base_link`` and ``odom`` encodes the robot's position according to its odometry, we can think of our robot's position in the map as being encoded by the transform between ``base_link`` and ``map``.  Rather than thinking of two independent transforms: one from ``base_link`` to ``map`` and one from ``base_link`` to ``odom``, we will instead think of a chain of transforms that lead us from ``map`` to ``odom`` to ``base_link``.

By considering a chain of transforms, the transform from ``base_link`` to ``map`` can be computed by first applying the transform from ``base_link`` to ``odom`` and then applying the transform from ``odom`` to ``map``.  In this way, the position of the robot in the map is affected by our wheel odometry (which is good since it is fast and always available), but we can also affect the position by changing the ``map`` to ``odom`` transform.  Changing the map to ``odom`` transform is exactly what we are going to do with our particle filters! 

### Computing the ``map`` to ``odom`` transform

In computing the ``map`` to ``odom`` transform, you will start with two things.
 1. The pose of the robot in the ``odom`` frame (think of this as the ``base_link`` to ``odom`` transform) let's call this transform $$T_{base\_link \rightarrow odom}$$.
 2. The pose of the robot in the ``map`` frame (e.g., as calculated by your particle filter).  We can think of the pose of the robot in the ``map`` frame as encoding the ``base_link`` to ``map`` transform, which we can call $$T_{base\_link \rightarrow map}$$.

If we think of these transforms as matrices (e.g., [homogeneous transformation matrices](http://planning.cs.uiuc.edu/node99.html) as we saw in [our second meeting](day02)), then the following must hold.

$$\begin{align}T_{odom \rightarrow map} T_{base\_link \rightarrow odom} &= T_{base\_link \rightarrow map} \\
T_{odom \rightarrow map} &= T_{base\_link \rightarrow map}T_{base\_link \rightarrow odom}^{-1} \\
T_{odom \rightarrow map}^{-1} &= \left(T_{base\_link \rightarrow map}T_{base\_link \rightarrow odom}^{-1} \right)^{-1} \\
T_{map \rightarrow odom} &= T_{base\_link \rightarrow odom} T_{base\_link \rightarrow map}^{-1}
\end{align}$$

In this way we have shown how to take the pose of the robot in the odom and the map frame and use it to compute the ``map`` to ``odom`` transform.  Note that this is a computation that is available in ``helper_functions.py``, but we wanted to give you a sense of what's going on in the function ``fix_map_to_odom_transform`` in ``helper_functions.py``.

Once the ``map`` to ``odom`` transform is computed, your particle filter should publish it repeatedly in your run loop in-case another node is waiting on the transform to become available.

## Estimating Robot Motion Using ``tf2``


In the warmup project some of you used the ``/odom`` topic as a way to figure out the position of your robot (according to its odometry).  By now we know that the for our particle filter we'll need to apply our motor model to update our particles.  One way to imagine doing would be to listen to the ``cmd_vel`` to see what velocities are sent to the robot.  Instead of doing this we can use our robot's odometry as a baseline to apply our motor model update.

In order to use our odometry to do our motor model update, we have two choices.  First, we could listen to the ``odom`` topic and try to track how much the position and orientation has changed since the last time we did an update.  Second, we can use ROS's ``tf2`` module as a way to compute how much the robot has moved between the last time we did an update and the current time.

> Let's quickly discuss what some advantages might be of the using the ``tf`` module.

We have put together some code to demonstrate how ``tf2`` can be used to estimate the relative motion of the robot between two points in time.  We'll leave it to you to decide when / if to go through the example.  To run the code, run the following command (make sure to do a ``git pull upstream master`` in the ``comprobo20`` directory to make sure you have this code).

```bash
$ roslaunch neato_gazebo neato_empty_world.launch
```

In a new terminal, run the following command and start the robot driving around (e.g., in a straight line by pressing ``i`` or in a donut by pressing ``u``).
```bash
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

In a new terminal, run the code for today.
```bash
$ rosrun in_class_day08 relative_motion.py
```

## Some Additional Advice

* Don't update your particles too often:  Typically you only want to update your particle set when the robot has moved a little bit or rotated a little bit.  This will prevent your filter from too aggressively incorporating sensor measurements from the current moment in time.
* Start with the easier case: try to track your robot given that you have accurate knowledge of its initial position.  You should not be trying to localize a robot when you have no idea where it is in the map right off the bat (and it's okay if you never solve this case!).
* **Visualizations:** Consider publishing weighted arrows to communicate the weights of the particles before resampling.

## Particle Filter Implementation Plan

Join up with another team and compare your implementation plans.  Try to identify parts of these plans that are fuzzy.  This would be a great time to ask clarifying questions of the teaching team.

## Discussion on Legal Issues in Robotics
* Link to <a-no-proxy href="https://docs.google.com/presentation/d/1TLTV-q67P7cgTb09ho6nzXvyprFYKCk9ozQm4j6gHRU/edit#slide=id.g9dea26f006_1_5">Robots - Legal Landscape</a-no-proxy> slides. 