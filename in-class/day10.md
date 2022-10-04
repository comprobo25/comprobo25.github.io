## Checking in on Particle Filter Project

## For Next Time

* Continue to work on your particle filter project.  Note that the project is due 10/12.

## So... how's it going?

Here is [a survey to figure out how things are going](https://docs.google.com/forms/d/e/1FAIpQLSer2aQ7ubTs8bIWNrPNT2MX02Kkz9Q9eLWxkhHQznK09EzLFQ/viewform).

## ``update_particles_with_odom``

You need to think about what ``delta`` is.  Draw a picture.  If you are just adding the entries of ``delta`` to the corresponding field of your partifles, you are not doing it correctly.


## ``update_particles_with_laser``

To be clear, we are suggesting you implement the likelihood (or beam) model.  Here are some resources.

1.  Consider consulting [Probabilistic Robotics](https://docs.ufpr.br/~danielsantos/ProbabilisticRobotics.pdf).  Specifically 6.4 contains the information about likelihood fields.

2. Sarah Sebo (Olin alum) has some [nice resources](https://classes.cs.uchicago.edu/archive/2021/winter/20600-1/class_meeting_06.html) that [break down the particle filter likelihood a bit more simply](https://github.com/intro-robotics-uchicago/class_meeting_06_likelihood_field).  

## Possible Extensions

Here are some possible extensions (from the survey).  In the open work time we can provide more detail on how these might work.

* I want to solve the robot kidnapping problem (unknown starting location)
* I want to reimplement the parts of the filter that were written for me (interactions with ROS)
* I want to experiment with laser scan likelihood functions
* I want to try to implement ray tracing instead of the likelihood field
* I want to understand in greater detail the connection between Bayes' filter and the particle filter
* I want to make my particle filter more computationally efficient
* I want to experiment with landmark-based likelihood functions
