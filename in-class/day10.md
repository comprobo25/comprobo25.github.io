---
title: "Search And Rescue Algorithms II // Computing Motion and Likelihoods"
toc_sticky: true
toc_data:
  - title: Today
    link: in-class/day10/#today
  - title: For Next Time
    link: in-class/day10/#for-next-time
  - title: Search And Rescue Algorithms II
    link: in-class/day10/#search-and-rescue-algorithms-ii
  - title: Computing Motion
    link: in-class/day10/#computing-motion
  - title: Likelihood Models
    link: in-class/day10/#likelihood-models
  - title: Possible Extensions
    link: in-class/day10/#possible-extensions
---

## Today
* Search and Rescue Algorithms II (For your consideration)
* Studio Time and Breakout Sessions -- 
  * Computing Relative Motion
  * Likelihood Models
* Possible Project Extensions (For your consideration)
 
## For Next Time
* Work on the [Robot Localization project](../assignments/robot_localization)
  * Demos due on **Thursday October 17th In-Class**
  * Code + Writeups due on **Friday October 18th 7PM**
* Read over the [Broader Impacts assignment Part 2](../assignments/broader_impacts), due on **November 5th at 7PM** 
  * Note -- discussions will happen on October 28th, October 31st, and November 4th; stay tuned!
* Consider whether there is [feedback you'd like to share about the class](https://forms.gle/giCwA1pkr4y3e4T37)
* **Extra Credit** An assignment to [reinforce probability fundamentals](../assignments/probability_basics/assignmentprobability_basics.pdf) is available to complete for extra credit (to be applied to the state estimation and localization unit). Due on **Friday October 18th 7PM** for those interested; [check the Canvas page for submission instructions](https://canvas.olin.edu/courses/822/assignments/13050).



## Bayesian Filtering and the Particle Filter

> Legacy notes about Bayes Filters and the Particle Filter from Paul Ruvolo are available: [as a video lecture](https://www.youtube.com/embed/l7CrjOTlioU) and as [physical notes](updated_bayes_filter.pdf). 

For your projects, you're implementing a particle filter, which is a subclass of algorithm under the more general category of _Bayesian filters_. A Bayesian filter is a recursive, or sequential, algorithm -- for localization, this means that the robot's state estimate is refined iteratively as observations or actions are taken.

There is a bit of vocabulary to know before we get started:
* Markov process: a chain of events in which the probability of each event depends only on the state of the previous event ("what happens next only requires me to think about what's happening now")
  * This is a useful _assumption_ about the way the world works, because now we don't have to consider the entire history of a robot, just what happened most recently.
* Monte Carlo algorithms: repeated random sampling is used to estimate a solution to a complex (often nonlinear) problem

We're going to walk through the steps of the Bayesian filter:
```
Steps of a Bayesian Filter:
1) Initialize with an estimate of the first pose
2) Take an action, and predict the new pose based on the motion model
3) Correct the pose estimate, given an observation
4) Repeat steps 2 and 3, ad nauseum (or until your robot mission is over)
```

### Prediction
During the prediction step, the current estimated pose of the robot is updated based on a _motion model_. The motion model captures how a control input may be mapped to the real world (what noise may be applied, for instance). Prediction will always increase the uncertainty we have about where the robot is in the world (unless we have perfect motion knowledge). Prediction asks: given where I think I am, where will I end up after I take this action?

### Correction
To reduce (or attempt to reduce) our uncertainty, we can look around us with an _observation model_ (which will also capture noise in our measurements). Correction asks: given what I am measuring, what is my likely pose based on my estimate of where I may be?

### Mathematical Details
We'll walk through the mathematical details of this for a simple world in which a robot can open and close a door, and can measure whether a door is open or closed. This example is borrowed from [Probabilistic Robotics](https://docs.ufpr.br/~danielsantos/ProbabilisticRobotics.pdf); a highly influential book in modern robotics.

### The Particle Filter
A Bayesian filter, in its purest form, asks us to work with continuous probability distributions, and that is computationally challenging (nigh intractable) most of the time for practical robotics problems. The particle filter addresses these computational challenges by allowing us to _draw samples from our probability distributions_ and apply our prediction and correction steps to each of those samples in order to get an empirical estimate of a new probability distribution. In this way, the particle filter is a Monte Carlo algorithm, and leverages the law of large numbers to "converge" towards the optimal answer. (You can get a sense about why sampling works to find complex distributions by [playing with this applet](http://chi-feng.github.io/mcmc-demo/app.html?algorithm=GibbsSampling&target=banana)).


## Bayes Filter (Continued) and Particle Filter Applications
Last time, we were looking at Bayes filters -- a class of algorithms (of which our particle filter is one!) that allows us to propagate our uncertainty about the world in time, and constrain it by using our observations. We're going to finish our discussion, and specifically think about how we actually deploy Bayesian Filter of any flavor on a real robot, and what it's implications are.

> Slides walking through our in-class derivation [here](https://docs.google.com/presentation/d/1ekeHfD7YOJLc6mHo8z4BHfnu2JCXtsvNfszsGDPM4Js/edit#slide=id.p), and a previous recording of the walkthrough by Paul Ruvolo can be watched [here](https://www.youtube.com/embed/l7CrjOTlioU).

Questions for consideration:
* What are the computational limitations of a Bayes Filter? How does particle filtering overcome those limitations?
* What are possible limitations of a particle filter? How might you overcome those limitations?
* How are you going to represent your motion model for the project?
* How are you going to represent your sensor model for the project?







## Search and Rescue Algorithms 1: SLAM
_Simultaneous Localization and Mapping_ or SLAM, is perhaps one of THE most quintessential algorithms in modern robotics today. The premise is simply that we would like the robot to know where it is _even if there is not a reference map_. In search and rescue applications, this is a powerful skill -- since even in urban environments there is potential for the landscape to have changed sufficiently that a known map is worthless. 

But how does a robot know where it is...without knowing where it is?

[Cyrill Stachniss](https://scholar.google.com/citations?user=8vib2lAAAAAJ&hl=en) has a quick 5-minute overview of SLAM worth a watch (FWIW he is also involved in some of the most widely used SLAM algorithms and computational representations in robotics used today): [Explainer](https://youtu.be/BuRCJ2fegcc).

As outlined in the video, there are two implemented pieces to a SLAM algorithm -- a front-end (takes the data and turns it into some useful representation), and a back-end (which tries to compute the location of the robot within a map).

The three different backend approaches mentioned -- EKF, Particle, and Graph-based SLAM -- are not only different implementations/techniques, but also represent both _online_ and _offline_ techniques. Online techniques are computed in practical time while a robot is underway. Offline techniques are generally much slower to compute; maybe even computed after a robot has run a mission. All of these methods can be used online (in some fashion), but it's worth noting that Graph-Based approaches are also often run _offline_.

Let's take a look at the conceptual difference between:
* an [EKF-based online approach](https://citeseerx.ist.psu.edu/document?repid=rep1&type=pdf&doi=4fcf1d9c8c0f86ba318738c6531bbdcfb094f85b) (take a look at page 10)
  * watch a simple demo [here](https://www.youtube.com/watch?v=vGXQ537gHCg)
* a [graph-based offline approach](http://ais.informatik.uni-freiburg.de/teaching/ss13/robotics/slides/16-graph-slam.pdf) (slides 1-10 give a reasonable gist)
  * watch a simple demo [here](https://www.youtube.com/watch?v=E6IvbjZA7Ao) (note when the map "squiggles")

> For way more math, have a look at this [Graph-Based SLAM tutorial paper](http://www2.informatik.uni-freiburg.de/~stachnis/pdf/grisetti10titsmag.pdf).


Some questions to consider:
* What may be some limitations to each of these SLAM algorithms?
  * When might an offline algorithm be appropriate? When not? 
  * When might an online algorithm be appropriate? When not?
* In what scenarios or environments do you think any SLAM algorithm will have a hard time navigating in?
* Which steps in the pseudocode do you want to learn more about?
* What might a SAR designer want to consider when selecting a SLAM algorithm to use? 


## Search And Rescue Algorithms II: EKF SLAM (For your consideration)
Last time we briefly talked about two flavors of SLAM algorithms, Kalman Filtering (or EKF) based, and Graph Based. Today, here is a bit more detail about EKF-based SLAM algorithms, for your consideration. Resources you may find useful include [this presentation by Cyrill Stachniss](http://ais.informatik.uni-freiburg.de/teaching/ws12/mapping/pdf/slam04-ekf-slam.pdf), and Chapter 10 of [Probabilistic Robotics](https://docs.ufpr.br/~danielsantos/ProbabilisticRobotics.pdf).

EKF SLAM is one of the earliest versions of a SLAM algorithm, used to solve the online SLAM problem. It requires:
* Feature-based maps (the terminology typically used is _landmarks_)
* That Gaussian noise is a reasonable assumption for motor and sensor uncertainty

### Goal of EKF SLAM
We've been focused on _localization_, which asks: where is my robot, given that I have a map? We've been representing this as a probability distribution:

$$bel(x_t) = \mathbb{P}(x_t|u_{1:t},z_{1:t})$$

where $$x_t$$ is my pose at some time $$t$$, and $$u$$ and $$z$$ are my robot actions and observations, respectively. Implicitly, we also have access to a map in this formulation. In SLAM, we have to consider that our _map is also unknown_ and so we want to solve a joint probability:

$$bel(x_t, \mathcal{M}) = \mathbb{P}(x_t,\mathcal{M}|u_{1:t},z_{1:t})$$

The map in our EKF world is a list of landmarks which are represented as poses in the world, each with uncertainty associated with them. Extended Kalman Filtering (EKF) allows us to track the uncertainty in our robot pose as well as each of our landmark poses; using a prediction and update step as established by a Bayesian filter. 

### Implementing EKF SLAM -- an interactive Python Notebook
Here is a [CoLab Notebook](https://colab.research.google.com/drive/1Sbm81zccVfqPNaV6w9rdGH56F1uCTUIF?usp=sharing) with a minimal EKF SLAM solution implemented for your review. (You will likely want to download this notebook and run it directly on your system, or from your particular CoLab account). Some key things to pay attention to as you review the code:
* What are the steps of the EKF SLAM algorithm? How do those compare to your Particle Filter? What's different?
* How does changing the noise parameters at the top impact how EKF-SLAM tracks the vehicle? The landmarks?

### Limitations and Design Considerations
* Absence of landmarks is not information that can be incorporated
* It's a linearization-based approach (but we live in a non-linear world)
* It has scaling challenges associated with the number of landmarks / size of a map
* Landmark detection and data association are _critical_ in EKF SLAM; ambiguous landmarks make this challenging
* What else?

### Connecting Back to Search and Rescue
* In what settings do you think that EKF SLAM may be an appropriate SLAM solution? In what settings would it be unsuitable?
* What risks may be associated with applying EKF SLAM to a search and rescue scenario? How might those risks be mitigated?
* What are key advantages to EKF SLAM?


