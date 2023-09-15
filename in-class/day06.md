## Today

* Warm-Up Project Debrief
* Conceptual introduction to the particle filter
* We're all living in a 1D world! (a simple particle filter example)

## For Next Time

* Read over the description of the [Robot Localization project](../assignments/robot_localization) (there are a few modifications we still need to make, but nothing that will really impact an initial readthrough).
* For more reinforcement of the concepts behind the particle filter, watch this <a-no-proxy href="https://www.youtube.com/watch?v=aUkBa1zMKv4">video</a-no-proxy>.

## Warmup Project Debrief

* Let's go through the [slide deck to share what you've done](https://docs.google.com/presentation/d/1zDFkOsO9zwGbvW_Cppq5NwiwWXBKT_F3Vgvbxg6CJjE/edit?usp=sharing).  Successes as well as hard-won lessons are equally welcome. 

## Conceptual Introduction to the Particle Filter

We'll be doing an activity to introduce our next major topic in the class: robot localization.  This is supposed to be a fun activity to get you thinking about the basic concepts.


## We're all living in a 1D world!

Before diving into this on your own, I want to show you some basic ideas in front of the class.  The instructions for running thing are summarized below.

To get the code for today you will need to make sure your environment is setup with matplotlib and scipy. If you want to check you can use

{% include codeHeader.html %}
```bash
pip3 show matplotlib scipy
```

If you get any warnings about package(s) not found, you can install them with ``pip3``.  For example, if I didn't have either package, you can use the following command to install the necessary libraries.


{% include codeHeader.html %}
```bash
pip3 install matplotlib scipy
```

Additionally, if you haven't done so yet, clone [the class activities are resources repo](https://github.com/comprobo23/class_activities_and_resources) into your ``ros2_ws/src`` folder.  If you've already cloned it, make sure to do a ``git pull origin main``.

Next, make sure to build your workspace and source your ``install/setup.bash`` file.

```bash
$ cd ~/ros2_ws
$ colcon build --symlink-install
$ source install/setup.bash
```

To try things out, let's first startup a 1d simulation of the world. 

{% include codeHeader.html %}
```bash
ros2 run simple_filter simple_filter_world.py --ros-args -p walls:=[0.0,3.0]
```

Take a look at the topics that are being published.  What types of messages are there?  What topics correspond to which messages?  We'll go through this as a class.

Next, we will experiment with our first particle filter:

{% include codeHeader.html %}
```bash
ros2 run simple_filter simple_particle_filter.py --ros-args -p walls:=[0.0,3.0] -p nparticles:=100
```

A visualization should come up.  The visualization shows the position of all the particles, the particle weights, and the true position.  Additionally, a histogram is generated that shows the belief about where the robot is.

You can move your robot around using the following ROS node.  To use this node make sure the window that pops up has focus, and use the ``a`` key and the ``d`` keys to move around left to right, respectively (sorry, I used to use the arrow keys, but I was finding that it didn't work reliably,)

{% include codeHeader.html %}
```bash
ros2 run simple_filter simple_controller.py
```

What happens over time to your visualization?

Try different wall configurations to see what happens.  What happens as you change the number of particles?  What happens if the wall configuration of the simulator and the particle filter model don't match up? 

Construct a scenario where there is an inherent ambiguity in determining where the robot is.  How do you do this?  What happens when you run your particle filter.
