## Today

* Bayesian filtering and the particle filter

## For Next Time

* Work on the [Robot Localization project](../assignments/robot_localization). For the next class you should have done the part of the "Implementation Plan" part of the assignment.

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
