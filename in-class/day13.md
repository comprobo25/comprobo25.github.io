---
title: "Sustainability Vectors // Machine Vision Project Ideation"
toc_sticky: true
toc_data:
  - title: Today
    link: in-class/day13/#today
  - title: For Next Time
    link: in-class/day13/#for-next-time
  - title: Sustainability in Robotics
    link: in-class/day13/#sust-robots
  - title: Machine Vision Project Ideation
    link: in-class/day13/#machine-vision-ideation
---

## Today

* Introduction to Sustainability in Robotics
* Project Ideation, Team Formation, and Proposals

## For Next Time
* Work on the [Broader Impacts assignment Part 2](../assignments/broader_impacts), due on **November 5th at 7PM** 
  * Note -- discussions will happen on October 28th, October 31st, and November 4th; you have been [randomly assigned one of these days to lead a discussion](https://docs.google.com/spreadsheets/d/1t2wJVq1ryEH47zOyPqVHE0VHtDHGa2fm6ehskNi13aA/edit?usp=sharing). You may swap slots with someone on a different day, but you have to let an instructor know. Thanks!
* Read over the [Machine Vision Project Document](../assignments/computer_vision_project).
    * Project Proposal is due on **Tuesday October 22nd at 7PM**. We'll have some in-class time today to work on this.
* Consider whether there is [feedback you'd like to share about the class](https://forms.gle/giCwA1pkr4y3e4T37)

<!-- ## Reading a Technical Research Paper (For Your Consideration)
As you progress through this class, you may want to consider reading technical papers to gather ideas for algorithms to test, open-source code to integrate into your solutions, or academic perspectives for your Broader Impacts assignment. Technical writing is its own art form, as is reading a piece of technical writing. Here is how I read a technical paper when I'm trying to learn more about a certain technique or field:
* **Start with the Abstract and Intro, then read the Conclusion.** To get a real sense for what a research paper is really about (what its core contribution is, the motivation of the work, and how it is related to other work in the field broadly) you should consider reading it out of order. The Abstract, Introduction, and Conclusion sections will hit the key points of the paper, and help you understand what details you might want to learn more about as you read the rest.
* **Read Every Figure Caption.** In the best papers, figures serve as a pictorial representation of the methods and the results. 
* **Read the Results.** Get a sense for how an algorithm or method performs. Do you trust in the performance metrics? Are the results statistically significant? Do the results align with the motivation of the work? If the results don't pass a "sniff check" then you might consider moving on to another paper.
* **Read the Methods.** This is often the most technical parts of a technical paper, and may include pseudocode, mathematical derivations and proofs, or system diagrams. If you've gotten to this step, it's worth skimming this section first, then reading it a second time in considerably more detail. Marking up a print copy or PDF version is recommended -- taking notes helps to process the material and generate questions that you can follow-up on with other reading or experimentation of your own.
* **Read the Related Works.** This is where you see all the fundamental work that the algorithm you're reading about was built on (helpful if you want to learn more about some methods in the paper) and contemporary work that the algorithm may compare itself too (helpful if you want to look at different strategies to solve the same / similar problems).
* **Skim the Whole Paper End-to-End.** Finish reading a paper by giving it a final skim in order, paying attention to the headings, key contribution statements, and how each section feeds into the next. This gives you a nice sense of the complete body of the work, helps you reassess the Results and Conclusion sections with more information from the Methods section, and can help you summarize the paper in your notes.

 Here are a few technical papers on state estimation and localization (our next unit!!) that may be interesting launch-points to learning more about these topics:
 * [ORB-SLAM: A Versatile and Accurate Monocular SLAM System](https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=7219438&casa_token=WVatKv5yIlgAAAAA:hJr0vy9MMdOkC6xrXPaYf-Yn91nXjGZRsqraEj-6UrRg656-j564yYNUooPZ2EOqKGghpSVj66E) by Mur-Artal et al., 2015
 * [Real-Time Loop Closure in 2D LIDAR SLAM](https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=7487258&casa_token=G_B93iYLcQYAAAAA:dCwVkq4rnNN28ojorfB5krmbOr3oETCstdLSrSoSleAORaElFFhyrZ3j1xKx8gRifKlFBhbMiNo) by Hess et al., 2016
 * [Back to the Feature: Learning Robust Camera Localization from Pixels to Pose](https://openaccess.thecvf.com/content/CVPR2021/papers/Sarlin_Back_to_the_Feature_Learning_Robust_Camera_Localization_From_Pixels_CVPR_2021_paper.pdf) by Sarlin et al., 2021
 * [Autonomous Navigation System of Greenhouse Mobile Robot Based on 3D Lidar and 2D Lidar SLAM](https://www.frontiersin.org/journals/plant-science/articles/10.3389/fpls.2022.815218/full) by Jiang et al., 2022

 Technical papers can be searched for in Library databases, [Google Scholar](https://scholar.google.com/), professional organization archives of journals and/or conferences (e.g., [IEEE Xplore](https://ieeexplore.ieee.org/Xplore/home.jsp)), or in open archives (e.g., [arXiv](https://arxiv.org/) -- Note that open archives are open...so not all work here may be peer-reviewed!).  -->

## Sustainability in Robotics
As we kickoff a new module, we'll be examining a new contextual theme: sustianability. Sustainability is commonly applied to three key "vectors" -- 
* **People / Social** -- creating well-being of people and communities 
   * Some topics include, e.g., socioeconomic equality and equity, access to resources, fair governance, education, human rights, etc.
* **Planet / Environmental** -- preserving and protecting the natural environment 
   * Some topics include, e.g., biodiveristy, air/water/soil quality, climate regulation, natural resource management, etc.
* **Products / Economic** -- enabling and preserving long-term economic well-being 
   * Some topics include, e.g., resource management, efficiency, innovation, policy and social equity, financial stability, etc.

> Discussion Question (8 minutes): How do you think robots fit into each of these vectors (either as a tool, or as an industry itself)? Do you have some examples of robots or companies that you could map to these vectors?

Throughout this module, we'll be taking a look at how machine vision is adapted into a very particular type of robotic system: waste collectors / recyclers -- 
* Automated Sorting in Recycling Plants (e.g., [rStream](https://www.rstreamrecycling.com/) and a deep dive into [this paper](https://arxiv.org/pdf/2106.02740))
* Automated Waste Collection (e.g., [Ocean CleanUp](https://theoceancleanup.com/) and a deep dive into [papers on bespoke solutions](https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=9501299&casa_token=-zTd-OPUEI8AAAAA:CDg94g-rOfgxdaUqO5Vj1xQ2WmszSRgbV-gr9w-_aKnRxaWu9o1MQZ3BKTDD-JojNEMtaOoRTP0))

> Discussion Question (8 minutes): In what ways do you think machine vision may be used in waste sorting / cleaning? What design characteristics would these algorithms need in order to be used in these applications? 


## Machine Vision Ideation and Team Formation
Take a look at the [machine vision project assignment document](../assignments/computer_vision_project) -- today we'll be forming teams, brainstorming project ideas, and starting your project proposals.

Process:
* [5 min] Individually review your learning goals for the class and for this project
* [5 min] Create a sticky-note for each topic/theme you'd like to explore in this project. Consider:
   * Is there a particular application of machine vision you'd like to investigate?
   * Is there a particular class of algorithm you'd like to learn about?
   * Is there a particular algorithm you'd like to implement or dataset you'd like to use?
* [5 min] Gather with 2 other folks and rapid-sort your sticky-notes into clusters on a white board -- label your clusters
* [5 min] As a class, we'll identify 6 common themes and place them around the room
* [10 min] Choose a theme that interests you and go to it in the room; with the other folks in the group, discuss your learning goals and ideas you have related to that theme
   * You're not committing to anything yet! This is about exploring some areas and project ideas of interest
* [10 min] Pick another theme and go to it; repeat your discussions
   * You're not committing to anything yet! This is about exploring some areas and project ideas of interest
* [Rest of Class Time] Find a partner and begin to scope a project you'd like to complete. You can use the proposal guidelines to frame your discussion
