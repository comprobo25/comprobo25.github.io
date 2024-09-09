---
layout: splash
title: "A Computational Introduction to Robotics 2024"
header:
  overlay_image: "website_graphics/robo_splash.jpg"
  overlay_color: "#000"
  overlay_filter: "0.4"

feature_row_TODO:
  - image_path: https://img.youtube.com/vi/MFL4gd2IMm8/0.jpg
    alt: "A thumbnail image for a video.  The text QEA Olin College of Engineering appears on a textured blue background"
    title: "Key Features of QEA"
    excerpt: "QEA is a highly interdisciplinary, integrated course for teaching technical content."
    url: "#course-philosophy"
    btn_label: "More details about QEA"
    btn_class: "btn--primary"
  - image_path: website_graphics/faces_collage.png
    alt: "A 5 by 5 grid of face images with labels of whether each of them are smiling."
    title: "Computational Platform"
    excerpt: "Students use Python as a programming language during this module.  Use the button below to see sample code and other materials."
    url: "#supporting-resources"
    btn_label: "Supporting Resources"
    btn_class: "btn--primary"
  - image_path: website_graphics/eigface1_stretched.jpg
    alt: "A sample Eigenface calculated from the class dataset."
    title: "Module Overview"
    excerpt: "The module introduces fundamental ideas in linear algebra through a deep dive into creating a facial recognition system."
    url: "#module-details"
    btn_label: "Module Details"
    btn_class: "btn--primary"

feature_row_robot:
  - image_path: website_graphics/neato_overview.jpeg
    alt: "A picture of a Neato robotic vacuum with a custom remote control interface based on Raspberry Pi"
    excerpt: "The documentation describes both how to connect to the the physical robot or a simulator and how to build your own customized Neato.

    ### Student Facing Documentation\n
    * [Setting up Your Computer](How to/setup_your_environment), [Using the Neatos](How to/use_the_neatos), and [Using the Turtlebot 4](How to/use_the_turtlebot4)\n
    * [Useful Resources](useful_resources) and [Sample Code](Sample_code/sample_code)\n

    ### Teaching Team Documentation\n
    * [Shopping list](How to/shopping_list) and [Platform Conversion Instructions](How to/Platform Conversion Instructions.pdf)\n
    * [Raspberry Pi Setup](How to/setup_raspberry_pi)"

feature_row_warmup_project:
  - image_path: website_graphics/barrel_follow.gif
    alt: "The Neato robot in a simulated world tracking a barrel"
    excerpt: "
   The Warmup Project provides a scaffolded assignment for students to get up to speed with important concepts in ROS through implementing compelling behaviors on a robot.  The project emphasizes the establishment of good practices such as debugging techniques and visualization.

    ### Supporting Documents

* [Warmup Project Assignment Document](assignments/warmup_project)\n
* [Class-generated Tips and Tricks](https://docs.google.com/document/d/15u9fvz5TsuPaSnvE1h_dyiVO2Rk7YTIyNweKpGGwuTE/edit?usp=drive_link)\n
* [Troubleshooting](How to/troubleshoot)\n"

feature_row_robot_localization_project:
  - image_path: website_graphics/neato_gazebo.png
    alt: "The Neato robot in an empty, simulated world"
    excerpt: "
   The Robot localization project is a scaffolded assignment for students to learn about the particle filter algorithm.  Along the way the will learn some basics of Bayesian inference and some new ROS tools and workflows.

    ### Supporting Documents

* [Robot Localization Assignment Document](assignments/robot_localization)\n
* [Dude, where's my robot? A Localization Challenge for Undergraduate Robotics](https://dl.acm.org/doi/abs/10.5555/3297863.3297895)\n"

feature_row_robots_society:
  - image_path: website_graphics/neato_gazebo.png
    alt: "The Neato robot in an empty, simulated world"
    excerpt: "
   The goal of this assignment is to examine the effects robots are having on our world and what we can do to make that effect positive. (TODO: Better image)

    ### Supporting Documents

* [Student Led Discussions](assignments/ethics_discussion_assignment)\n"

feature_row_computer_vision_project:
  - image_path: website_graphics/neato_gazebo.png
    alt: "The Neato robot in an empty, simulated world"
    excerpt: "
The computer vision project is an open-ended project on using computer vision in the context of robotics.

    ### Supporting Documents

* [Computer Vision Project Document](assignments/computer_vision_project)\n"

feature_row_final_project:
  - image_path: website_graphics/neato_gazebo.png
    alt: "The Neato robot in an empty, simulated world"
    excerpt: "
   The final project is an open-ended project that lets students explore a robotics topic and algorithms in depth.

    ### Supporting Documents

* [Final Project Assignment Document](assignments/final_project)"

---

The Olin College course "A Computational Introduction to Robotics" (CompRobo) serves as a tour through some of the most important ideas at the heart of modern robotics.  The course utilizes a project-based learning pedagogy that allows students to build mastery of key concepts while also allowing for a great deal of student choice and autonomy.  The major focal points of the course are state estimation, localization, computer vision, decision-making, and societal implications of embodied systems. 

<!-- {% include feature_row %}-->

## <a name="robot-details"/> Robot Details and Documentation

{% include feature_row id="feature_row_robot" type="left" %}

## <a name="module-details"/> Warmup Project: Software Development for Robots

{% include feature_row id="feature_row_warmup_project" type="right" %}

## <a name="module-details"/> Robot Localization Project

{% include feature_row id="feature_row_robot_localization_project" type="left" %}

## <a name="module-details"/> Robots in the World

{% include feature_row id="feature_row_robots_society" type="right" %}

## <a name="module-details"/> Computer Vision Project

{% include feature_row id="feature_row_computer_vision_project" type="left" %}

## <a name="module-details"/> Final Project

{% include feature_row id="feature_row_final_project" type="right" %}

## In-class Activities

Note: see [Site-wide TOC for an easy to navigate outline of each day's activities](toc)

* [Day 1: Welcome!](in-class/day01)
* [Day 2: The Landscape of Modern Robotics // Basic ROS Concepts](in-class/day02)
* [Day 3: What are Broader Impacts? // Writing sensory-motor loops in ROS](in-class/day03)
* [Day 4: Reaching a Technical Research Paper // Debugging, Proportional Control, and ROS Parameters](in-class/day04)
* [Day 5: Frameworks for Challenging Discussions // Wall Bumping and Finite-State Control](in-class/day05)
* [Day 6: Conceptual Introduction and Simple 1D Particle Filter](in-class/day06)
* [Day 7: The Particle Filter for Robot Localization](in-class/day07)
* [Day 8: Robot State Estimation and Bayes](in-class/day08)
* [Day 9: Computing Relative Motion](in-class/day09)
* [Day 10: Gauss, Likelihood Fields, and the Particle Filter](in-class/day10)
* [Day 11: Particle Filter Debugging Strategies, Extensions, and Studio Day](in-class/day11)
* [Day 12: Studio Day and Introduction to Computer Vision Project](in-class/day12)
* [Day 13: Robot Localizaiton Shareout and Computer Vision Project Ideation and Team Formation](in-class/day13)
* [Day 14: Neato Soccer and Discuss Project Proposals](in-class/day14)
* [Day 15: Broader Impacts Discussion Session 1 // Keypoint Matching and Descriptors](in-class/day15)
* [Day 16: Broader Impacts Discussion Session 2 // Project Work Time](in-class/day16)
* [Day 17: Broader Impacts Discussion Session 3 // Project Work Time](in-class/day17)
* [Day 18: Project Work Time](in-class/day18)
* [Day 19: Computer Vision Showcase and Final Project Kickoff](in-class/day19)
* [Day 20: Project Proposal Generation](in-class/day20)
* [Day 21: Planning Under Uncertainty // Project Work Time](in-class/day21)
* [Day 22: Project Proposal Generation](in-class/day22)
* [Day 23: Cognitive Robots // Project Work Time](in-class/day23)
* [Day 24: Project Work Time](in-class/day24)
* [Day 25: Project Work Time](in-class/day25)
* [Day 26: Project Work Time](in-class/day26)
* [Day 27: Final Project Showcase and Semester Reflection](in-class/day27)


## Conclusion and Learning More
CompRobo serves as a fun, hands-on introduction to key ideas in robotics algorithms and toolsets.  Despite the fact that the course is successful at Olin, we realize that everyone's institutional context is different. To connect with folks at Olin College to learn more about this module or determine how you might build off of this at your own institution, e-mail <a href="mailto:Collaboratory@olin.edu">Collaboratory@olin.edu</a> to start the conversation.
