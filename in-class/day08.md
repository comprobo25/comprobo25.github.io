## Today

* Wrap on last week's lecture
* Discussion on Legal Issues in Robotics
* Debrief on software architecture plans for particle filter
* Practical considerations for particle filters

## For Next Time
* Work on the <a-no-proxy href="https://olin.instructure.com/courses/143/assignments/1325">Robot Localization project</a-no-proxy>.
* Disscussion Readings

## Discussion on Legal Issues in Robotics

TODO

## Particle Filter Implementation Plan

Join up with another team and compare your implementation plans.  Try to identify parts of these plans that are fuzzy.  This would be a great time to ask clarifying questions of the teaching team.

## The Particle Filter and Coordinate Frames

When we think of robot localization, there are usually two ways that robots estimate motion.
* **Odometry:** robot's use sensors to determine their motion over short windows of time and use the dead-reckoning to estimate their motion.  Robots typically utilize sensors such as wheel encoders (that measure how far wheels turn), accelerometers, and gyroscopes for their odometry.
  * Pros: very fast to update (typically requires minimal computation), updates can always be performed (these sorts of sensors usually always give usable data).
  * Cons: errors accumulate over time, estimates can be very inaccurate sometimes (e.g., wheel slippage). 
* **Landmarks:** robot's can detect landmarks in their environment as a way to determine their position.  Landmark-based motion estimation is able to cancel out errors that have accumulated in the odometry-based motion estimation by referencing the current position to the robot's current position to its position at some other, potentially distant point in time.
  * Pros: Able to cancel out accumulated error in position.
  * Cons: often computationally expensive and there are not always suitable landmarks to use for such motion estimation (i.e., landmarks may only be intermittently available).

When implementing our particle filter we'd like to get the best of both words: the fast and always available updates of odometry and the ability to cancel out accumulated motion estimation error using landmarks.  We can use the concept of coordinate frames in ROS as a way to accomplish this.

### The map->odom->base_link chain

In the warmup project we met the coordinate frames ``odom`` and ``base_link``.  Recall that ``base_link`` is a coordinate system that rides around with the Neato.  Also recall that ``odom`` is a coordinate system that initially starts aligned with ``base_link`` (meaning the robot starts at the origin of ``odom``) and the relationship between ``odom`` and ``base_link`` is constantly updated to encode the robot's estimate of its position relative to its starting location using its odometry. 

In the particle filter project we are introducing an additional coordinate frame called ``map``.  The ``map`` frame is defined in terms of a map that our particle filter code is given.  Just as the transform between ``base_link`` and ``odom`` encodes the robot's position according to its odometry, we can think of our robot's position in the map as being encoded by the transform between ``base_link`` and ``map``.  Rather than thinking of two independent transforms one from ``base_link`` to ``map`` and one from ``base_link`` to ``odom``, we will instead think of a chain of transforms that lead us from ``map`` to ``odom`` to ``base_link``.

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

### Estimating Robot Motion Using ``tf2``

[http://wiki.ros.org/tf2/Tutorials](Tf2 Tutorials)

## Some Additional Advice

* Don't update your particles too often:  Typically you only want to update your particle set when the robot has moved a little bit or rotated a little bit.  This will prevent your filter from too aggressively incorporating sensor measurements from the current moment in time.
* Start with the easier case: try to track your robot given that you have accurate knowledge of its initial position.  You should not be trying to localize a robot when you have no idea where it is in the map right off the bat (and it's okay if you never solve this case!).
* Add visualizations: For example, you might want to publish weighted arrows to communicate the weights of the particles before resampling.
* Get comfortable with using the odometry to get the robot's motion: you'll have to calculate the motion to apply to each particle based on the readings you get from your odometry.
