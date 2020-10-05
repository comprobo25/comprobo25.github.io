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

### Estimating Robot Motion Using ``tf2``

[http://wiki.ros.org/tf2/Tutorials](Tf2 Tutorials)

### Publishing the ``map`` to ``odom`` transform

## Models of Laser Scans

### Modeling the Likelihood of Each Measurement

### Combining Multiple Measurements

## Some Additional Advice

* Don't update your particles too often:  Typically you only want to update your particle set when the robot has moved a little bit or rotated a little bit.  This will prevent your filter from too aggressively incorporating sensor measurements from the current moment in time.
* Start with the easier case: try to track your robot given that you have accurate knowledge of its initial position.  You should not be trying to localize a robot when you have no idea where it is in the map right off the bat (and it's okay if you never solve this case!).
* Add visualizations: For example, you might want to publish weighted arrows to communicate the weights of the particles before resampling.
* Get comfortable with using the odometry to get the robot's motion: you'll have to calculate the motion to apply to each particle based on the readings you get from your odometry.
