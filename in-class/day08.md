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

Join up with another team and compare your implementation plans. 

Some additional questions to consider:
* What classes will you create for your implementation (everyone should have a ParticleFilter class as given in the starter code, but more are probably better)?
* What functions will be implemented in these classes?
* If you will be using the code in the helper classes, how will your code utilize them?
* Define a timeline of how the code will interact with the sensor data coming from the robot and generate update estimates of the robot's location.

## ROS Infrastructure to Support the Particle Filter

### Estimating Robot Motion Using ``tf2``

[http://wiki.ros.org/tf2/Tutorials](Tf2 Tutorials)


### Publishing the ``map`` to ``odom`` transform

## Some General Advice

* Don't update your particles too often:  Typically you only want to update your particle set when the robot has moved a little bit or rotated a little bit.  This will prevent your filter from too aggressively incorporating sensor measurements from the current moment in time.
* Start with the easy case: try to reproduce the functionality of the built-in particle filter.  You should not be trying to solve the robot localization project right off the bat.
* Make sure you understand the intuition behind likelihood fields: the sample code for likelihood fields should provide a pretty straightforward path to implementing your measurement likelihood function.
* Add visualizations: For example, you might want to publish weighted arrows to communicate the weights of the particles before resampling.
* Get comfortable with using the odometry to get the robot's motion: you'll have to calculate the motion to apply to each particle based on the readings you get from your odometry.
* Use bag files to increase the repeatability of your testing: if your particle filter is doing something weird, make a bag file and use the bag file to debug your code.

## Additional Resources - OLD FROM WEBSITE
Probabilistic Robotics by Thrun, Burgard, and Fox has some great material on the particle filter algorithm in general and the motion and measurement models.  This book is available in the library (it is on reserve for this class).  The relevant chapters are 5 (motion models) and 6 (measurement models) and 4.2 (particle filter algorithm).  You can certainly consult this material to implement your particle filter, however, I do not think that everyone must do so.  There are some useful rules of thumb that you can keep in mind when implementing your filter.
