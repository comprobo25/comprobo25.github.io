---
title: "Simulators"
toc_sticky: true
---
In the beginning of the class you'll be mainly working wth [the Neato simulator](../How to/run_the_neato_simulator).  However, there will be lots of opportunities to use other simulators for other projects.

## Prius Simulator
The Open Source Robotics Foundation (OSRF) put together a demo of a Prius that can be controlled through ROS.  Notably,
the car is outfitted with many sensors that resemble what you'd find on an autonomous car (e.g., 3D lidar, many cameras).
An overview of the demo is available on [this blog post](https://www.osrfoundation.org/simulated-car-demo/).

We got the demo running and collected a few screenshots.  First, this is what the view of the Prius looks like through Gazebo.

![A view of a simulated Prius](../website_graphics/car_demo_gazebo.png)

Second, here is a visualization of the car's sensor data through rviz.

![A visualization of the Prius' sensor data](../website_graphics/car_demo_rviz.png)

### Setting up the Simulator Prerequisites

This one is a bit involved, but we did get it to work successfully.

#### Get the Code
First, clone the [car_demo](https://github.com/osrf/car_demo) repository.  Avoid cloning it under your `catkin_ws` directory as it will interfere with
running `catkin_make`.

#### Get Docker

Getting the demo running is fairly involved since this demo has not been updated for ROS Noetic (or even ROS Melodic for that matter).  To get around
this the authors of the demo have included a procedure to build and run the demo through Docker.  As a firs step, you should install Docker and perform the post-install steps for Linux.  The relevant instructions can be found on the Docker website under [install using the repository](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository).

Once you've installed Docker, you'll want to setup docker so you can run it as a non-root user.  TO do this you need to perform the following steps.

```bash
$ sudo groupadd docker 
$ sudo usermod -aG docker $USER
$ newgrp docker
```

To verify that everything is working, run the following command.
```bash
docker run hello-world
```

If everything is good to go, you should see output that resembles the following.
```bash
Unable to find image 'hello-world:latest' locally
latest: Pulling from library/hello-world
0e03bdcc26d7: Pull complete 
Digest: sha256:7f0a9f93b4aa3022c3a4c147a449bf11e0941a1fd0bf4a8e6c9408b2600777c5
Status: Downloaded newer image for hello-world:latest

Hello from Docker!
This message shows that your installation appears to be working correctly.

To generate this message, Docker took the following steps:
 1. The Docker client contacted the Docker daemon.
 2. The Docker daemon pulled the "hello-world" image from the Docker Hub.
    (amd64)
 3. The Docker daemon created a new container from that image which runs the
    executable that produces the output you are currently reading.
 4. The Docker daemon streamed that output to the Docker client, which sent it
    to your terminal.

To try something more ambitious, you can run an Ubuntu container with:
 $ docker run -it ubuntu bash

Share images, automate workflows, and more with a free Docker ID:
 https://hub.docker.com/

For more examples and ideas, visit:
 https://docs.docker.com/get-started/
```

#### Install NVIDIA Container Toolkit

Get Docker setup to work with your NVIDIA GPU (assuming you are using your Olin laptop you should have one).  To get it working, run the following commands ([link to the original instructions](https://github.com/NVIDIA/nvidia-docker))
```bash
# Add the package repositories
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
```

#### Install Rocker

Rocker is a tool to streamline running Docker containers with 3D and GUI support.  You can install it using ``pip``.

```bash
$ pip3 install rocker
```

### Building the Simulator

Navigate to the directory where you clone ``car_demo`` and run the following command.

```bash
./build_demo.sh
```

If everything worked, you'll see the following output at end of running the command.
```bash
Successfully built 00dd0ec5e7c8
Successfully tagged osrf/car_demo:latest
```

### Running the Simulator

Run the following command to start the simulator.

```bash
$ rocker --nvidia --x11 --network host  --devices /dev/input/js0 /dev/input/js1 -- osrf/car_demo
```

If all is working, you should see an ``rviz`` window and ``gazebo`` window pop up that look similar to the screenshots earlier int his document.

### Controlling the Prius

We haven't done too much examination of this, but to get the car driving forward, run the following command.

```bash
$ rostopic pub /prius prius_msgs/Control "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
throttle: 2.0
brake: 0.0
steer: 0.0
shift_gears: 0" 
```

You should see the car moving in the Gazebo window.

If you find out more cool stuff through your own explorations of this simualator, please let us know (or leave an annotation here).

## PX4 Simulator


