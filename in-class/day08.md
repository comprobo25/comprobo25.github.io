## Today

* Bayesian filtering and the particle filter

## For Next Time

* Work on the [Robot Localization project](../assignments/robot_localization). For the next class you should have done the part of the "Implementation Plan" part of the assignment.

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

### The map->odom->base_footprint chain

In the warmup project we met the coordinate frames ``odom`` and ``base_footprint``.  Recall that ``base_footprint`` is a coordinate system that rides around with the Neato.  Also recall that ``odom`` is a coordinate system that initially starts aligned with ``base_footprint`` (meaning the robot starts at the origin of ``odom``) and the relationship between ``odom`` and ``base_footprint`` is constantly updated to encode the robot's estimate of its position relative to its starting location using its odometry. 

In the particle filter project we are introducing an additional coordinate frame called ``map``.  The ``map`` frame is defined in terms of a map that our particle filter code is given.  Just as the transform between ``base_footprint`` and ``odom`` encodes the robot's position according to its odometry, we can think of our robot's position in the map as being encoded by the transform between ``base_footprint`` and ``map``.  Rather than thinking of two independent transforms: one from ``base_footprint`` to ``map`` and one from ``base_footprint`` to ``odom``, we will instead think of a chain of transforms that lead us from ``map`` to ``odom`` and then to ``base_footprint``.

By considering a chain of transforms, the transform from ``base_footprint`` to ``map`` can be computed by first applying the transform from ``base_footprint`` to ``odom`` and then applying the transform from ``odom`` to ``map``.  In this way, the position of the robot in the map is affected by our wheel odometry (which is good since it is fast and always available), but we can also affect the position by changing the ``map`` to ``odom`` transform.  Changing the map to ``odom`` transform is exactly what we are going to do with our particle filters! 

### Computing the ``map`` to ``odom`` transform

In computing the ``map`` to ``odom`` transform, you will start with two things.
 1. The pose of the robot in the ``odom`` frame (think of this as the ``base_footprint`` to ``odom`` transform) let's call this transform $$T_{base\_footprint \rightarrow odom}$$.
 2. The pose of the robot in the ``map`` frame (e.g., as calculated by your particle filter).  We can think of the pose of the robot in the ``map`` frame as encoding the ``base_footprint`` to ``map`` transform, which we can call $$T_{base\_footprint \rightarrow map}$$.

If we think of these transforms as matrices (e.g., [homogeneous transformation matrices](https://www.mecharithm.com/homogenous-transformation-matrices-configurations-in-robotics/) as we saw in [our second meeting](day02)), then the following must hold.

$$\begin{align}T_{odom \rightarrow map} T_{base\_footprint \rightarrow odom} &= T_{base\_footprint \rightarrow map} \\
T_{odom \rightarrow map} &= T_{base\_footprint \rightarrow map}T_{base\_footprint \rightarrow odom}^{-1} \\
T_{odom \rightarrow map}^{-1} &= \left(T_{base\_footprint \rightarrow map}T_{base\_footprint \rightarrow odom}^{-1} \right)^{-1} \\
T_{map \rightarrow odom} &= T_{base\_footprint \rightarrow odom} T_{base\_footprint \rightarrow map}^{-1}
\end{align}$$

In this way we have shown how to take the pose of the robot in the odom and the map frame and use it to compute the ``map`` to ``odom`` transform.  Note that this is a computation that is available in ``helper_functions.py``, but we wanted to give you a sense of what's going on in the function ``fix_map_to_odom_transform`` in ``helper_functions.py``.

Once the ``map`` to ``odom`` transform is computed, your particle filter should publish it repeatedly in your run loop in-case another node is waiting on the transform to become available.

## Particle Filter and Computing Relative Motion

One part of your particle filter will involve [estimating the relative motion of your robot between two points in time as given by the robot's odometry](https://github.com/comprobo22/robot_localization/blob/47899d1d1745b56adace25fdff1d08a6bf253a8b/robot_localization/pf.py#L228).  Suppose you are given the robot's pose at time $$t_1$$ as a homogeneous transformation matrix (a 2x2 rotation matrix and a 2 dimensional translation vector) along with the robot's pose at time $$t_2$$, how might you compute your robot's change in pose between these two time points?  Think carefully about what coordinate system you want to work in and if you have time, provide an equation to comput the change in pose. 


> Note: while probably not needed for dealing with 2D rotation and translation, I have found the ``PyKDL`` library to be a great way to convert between various representations of orientation and transformations (e.g., see [this section of the starter code](https://github.com/comprobo22/robot_localization/blob/47899d1d1745b56adace25fdff1d08a6bf253a8b/robot_localization/helper_functions.py#L121)).


## Randomness and the 1D Particle Filter

At this point it's useful to think a bit about how the concept of randomness and the role it plays in the particle filter example.  There seems to be randomness in a few different dimensions.  The world itself has some randomness (the scans have some noise and so does the movement) and the particle filter itself has some concept of noise as well.  In the simulator, you'll see a lot of code that looks something like the following examples.

```python
self.position = randn()*0.2+1.5
```

```python
north_laser_reading = (closest_north - self.position) + self.noise_rate*randn() 
```

```python
self.odom_position += velocity*self.dt + self.odom_noise_rate*randn()
```

Let's take a few minutes to think about what these are doing.

When we look at the particle filter code, we also see randomness but it appears in a bit different way.

```python
for i in range(self.n_particles):
    self.pf.add_particle(Particle(position=randn()+1.5,
				  weight=1/float(self.n_particles),
				  sensor_model=sensor_model))
```

```python
def sample_prediction(self, predicted_position):
    """ Sample a potential next state based on a predicted position
        based off of the Odometry """
    return predicted_position + randn()*self.odom_noise_rate
```

```python
return norm(0, self.model_noise_rate).pdf(abs(position - closest) - observation)
```

Let's take a few minutes to think about what these are doing.

## Bayesian Filtering and the Particle Filter

* [Notes from Today on Bayes' filter](bayes_filter.pdf)
* Notes from Previous Years on Bayes' filter and how it applies to robot localization are <a-no-proxy href="https://drive.google.com/file/d/19sKAjnXwNeYJG45RLjHPRsiTbP8TuF7A/view">here</a-no-proxy>.

I will lead a walkthrough of the derivation in-class.  You are welcome to hangout in the room and listen (and hopefully ask questions), you can go right now and work with friends, you can leave once you have the basic idea, etc.

### Proposed Model for Working through this Derivation

With your partner and another team, work through the notes.  Here are some guidelines.

* Make sure everyone understands each step before moving onto the next one.
* If there is a persistent issue that cannot be resolved, call a member of the teaching team over and ask us for help.  If for some reason this is not easy to do at the time, make a note of it so you can return to it later.
* If you need more help at any point in the derivation, don't hesitate to call a member of the teaching team over and ask for help.
* Stop when you get to the section labeled "Scalability."

## The Particle Filter Algorithm (together as a class)

As a class, let's go through the basic steps of the particle filter algorithm and the issues around scalability of the Bayes filter.

<iframe width="560" height="315" src="https://www.youtube.com/embed/l7CrjOTlioU" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
