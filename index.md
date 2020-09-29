---
layout: splash
title: "A Computational Introduction to Robotics"
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
    alt: "A picture of a Neato robotic vacuum cleaner with a custom remote control interface based on Raspberry Pi"
    excerpt: "The documentation describes both how to connect to the the physical robot or a simulator and how to build your own customized Neato.

    ### Student Facing Documentation\n
    * [Setting up Your Computer](How to/setup_your_environment) and [Running the Neato Simulator](How to/run_the_neato_simulator)\n
    * [Useful Resources](useful_resources) and [Sample Code](Sample_code/sample_code)\n

    ### Teaching Team Documentation\n
    * [Shopping list](How to/shopping_list) and [Platform Conversion Instructions](How to/Platform Conversion Instructions.pdf)\n
    * [Raspberry Pi Setup](How to/setup_raspberry_pi)"

feature_row_warmup_project:
  - image_path: website_graphics/neato_gazebo.png
    alt: "The Neato robot in an empty, simulated world"
    excerpt: "
   The Warmup Project provides a scaffolded assignment for students to get up to speed with important concepts in ROS through implementing compelling behaviors on a robot.  The project emphasizes the establishment of good practices such as debugging techniques and visualization.

    ### Supporting Documents

* [Warmup Project Assignment Document](assignments/warmup_project)\n
* [Class-generated Tips and Tricks](https://docs.google.com/document/d/1qKx8a1RNRpeyYIiZdDyUJzNqXsaaCSz8rG0UKaqvYIo/edit?usp=sharing)\n"

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

---

The Olin College course "A Computational Introduction to Robotics" (CompRobo) serves as a tour through some of the most important ideas at the heart of modern robotics.  The course utilizes a project-based learning pedagogy that allows students to build mastery of key concepts while also allowing for a great deal of student choice and autonomy.  The major focal points of the course are mobile robotics, computer vision, and machine learning. This year we are also piloting student led discussions on societal issues and robotics. 

<!-- {% include feature_row %}-->

## <a name="robot-details"/> Robot Details and Documentation

{% include feature_row id="feature_row_robot" type="left" %}

## <a name="module-details"/> Warmup Project

{% include feature_row id="feature_row_warmup_project" type="right" %}

## <a name="module-details"/> Robot Localization Project

{% include feature_row id="feature_row_robot_localization_project" type="left" %}

## <a name="module-details"/> Robots in the World

{% include feature_row id="feature_row_robots_society" type="right" %}

## In-class Activities

* [Day 1](in-class/day01)
* [Day 2](in-class/day02)
* [Day 3](in-class/day03)
* [Day 4](in-class/day04)
* [Day 5](in-class/day05)
* [Day 6](in-class/day06)
* [Day 7](in-class/day07)

## Conclusion and Learning More


CompRobo serves as a fun, hands-on introduction to key ideas in robotics algorithms and toolsets.  Despite the fact that the course is successful at Olin, we realize that everyone's institutional context is different. To connect with folks at Olin College to learn more about this module or determine how you might build off of this at your own institution, e-mail <a href="mailto:Collaboratory@olin.edu">Collaboratory@olin.edu</a> to start the conversation.
