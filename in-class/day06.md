## Today

* Conceptual introduction to the particle filter
* We're all living in a 1D world! (a simple particle filter example)

## For Next Time

* Complete this assignment on basic probability <a-no-proxy href="https://olin.instructure.com/courses/143/assignments/1317">basic probability</a-no-proxy>. Submit your work on canvas. 
* Read over the description of the <a-no-proxy href="https://olin.instructure.com/courses/143/assignments/1325">Robot Localization project</a-no-proxy>
* For more reinforcement of the concepts behind the particle filter, watch this <a-no-proxy href="https://www.youtube.com/watch?v=aUkBa1zMKv4">video</a-no-proxy>.

## Conceptual Introduction to the Particle Filter
The particle filter as <a-no-proxy href="https://docs.google.com/presentation/d/1viXb11AmVX3hI53HkJfYveJRdm6aJJhD6JaNkc1xQNk/edit#slide=id.p">zoom art</a-no-proxy>. Find the lost robot at Olin. 

## We're all living in a 1D world!
To get the code for today you will need to make sure your environment is setup with matplotlib and scipy. If you want to check you can use 

```
$ pip3 list
```

 If these are not installed, try the following (if you are using Python 2.7, replace the first line with python-tk). For Melinda, matplotlib was installed, but not scipy. 

```
$ sudo apt install python3-tk
$ pip3 install matplotlib
$ python3 -m pip install scipy 
```

Additionally, make sure to pull the latest changes from the upstream comprobo repository.

```
$ cd ~/catkin_ws/src/comprobo20
$ git pull upstream master
```

To try things out, let's first startup a 1d simulation of the world. 

```
$ rosrun simple_filter simple_filter_world.py _walls:=[0.0,3.0]
```

Take a look at the topics that are being published.  What types of messages are there?  What topics correspond to which messages?  Take a moment with your partner and make a list of topics and what they might encode.

Next, we will experiment with our first particle filter:

```
$ rosrun  simple_filter simple_particle_filter.py _walls:=[0.0,3.0] _nparticles:=100 _realrobot:=False
```

A visualization should come up.  The visualization shows the position of all the particles, the particle weights, and the true position.  Additionally, a histogram is generated that shows the belief about where the robot is.

You can move your robot around using the following ROS node.  To use this node make sure the window that pops up has focus, and use the arrow keys to move around left to right.

```
$ rosrun simple_filter simple_controller.py
```

What happens over time to your visualization?

Try different wall configurations to see what happens.  What happens as you change the number of particles?  What happens if the wall configuration of the simulator and the particle filter model don't match up? 

Construct a scenario where there is an inherent ambiguity in determining where the robot is.  How do you do this?  What happens when you run your particle filter.

### Check out the Code
Next, drill down into the code make sure you take a good look at SensorModel.get_likelihood and SensorModel.sample_prediction.    The first function is determining the probability of a sensor measurement for a particular hypothesized robot pose.  What does this likelihood function look like? Draw lots of pictures.  What assumptions in this function would be violated with the actual Neato.

The second function samples a potential next state based on a measured change via the odometry.  What assumptions are encoded in this function.  Again, draw lots of pictures.

## Team Formation
We'll have some time for informal team formation at the end of class.  Everyone must fill out this form to indicate your teammate, or if you don't have one yet, you should provide me with some information to help match you.