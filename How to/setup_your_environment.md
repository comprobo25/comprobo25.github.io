---
title: "Setup Your Computing Environment"
toc_sticky: true
---

The teaching team will be using ROS Noetic with Ubuntu 20.04, but you should be able to use 18.04 and Melodic.  We will indicate where the instructions would differ if you were using Melodic.

> While there are other ways to install ROS on a computer (ROS for Windows, ROS through Docker, ROS through Windows Subsystem for Linux, ROS2), you really, really want to use Ubuntu running via dual boot (not as a virtual machine).  We have found that while these other setups work to varying degrees, there are always little issues that will crop up that will likely get in the way of your learning.  The biggest issue you will see is that most of these other setups will not allow you to run robot simulators with hardware-based graphics acceleration.  Given how much we will be using simulators this semester, you really want the superior performance that comes from hardware acceleration.


## Setting up a Dual Boot

In order to setup your computer for dual boot, you need to create a bootable USB thumb drive with Ubuntu 20.04 on it.  Itzgeek has [a nice walkthrough of how to do this](https://www.itzgeek.com/post/how-to-install-ubuntu-20-04-alongside-with-windows-10-in-dual-boot/) that will allow you to take an existing USB thumb drive and convert it into a bootable installer.

> Although we recommend Noetic, if you are using Melodic, you would want to install Ubuntu 18.04 instead.

A few quick notes:
* If you have Ubuntu 18.04, you can upgrade it 20.04 using [these instructions](https://ubuntu.com/blog/how-to-upgrade-from-ubuntu-18-04-lts-to-20-04-lts-today).
* When going through the Itzgeek tutorial, we recommend following the instructions under "Create a bootable USB disk".
* When installing Ubuntu you will likely need to **shrink your Windows patition** to make room for Ubuntu.  The Itzgeek instructions show how you can use the Disk Management utility in Windows to accomplish this.  Unfortunately, sometimes you will not be able to shrink your volume in this way.  If this happens to you, we recommend using the Ubuntu installer to shrink your Windows partition.  If you continue to have issues, we had success using [EaseUS Partition Manager](https://www.easeus.com/partition-manager/epm-free.html).  If that still doesn't work, send us an e-mail (see below).
* You should probably reserve about 50 GB of space for Ubuntu.
* When installing Ubuntu, you should select the options to **Download updates** and **Install third-party software**.


*Once you have a freshly installed copy of Ubuntu 20.04, perform the following steps.*

### Troubleshooting

One student reported an error message about needing to turn off RST to install Ubuntu.  The student was able to find a workaround.  If you have this or any other issue, please send an e-mail to <a href="mailto:comprobofaculty@olin.edu">comprobofaculty@olin.edu</a>.

## Install ROS Noetic

Follow [this tutorial](http://wiki.ros.org/noetic/Installation) (make sure to complete the steps for ``Ubuntu`` and ``ros-noetic-desktop-full``).

> If you are installing ROS Melodic, follow [this tutorial](http://wiki.ros.org/melodic/Installation/Ubuntu).
> Once you are done, complete the instructions at the end of this document under <a href="#melodic-python-3-support">Melodic Python 3 Support</a>. 

## Setup your Catkin Workspace

For more context on what is going on here, see [this ROS tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

```bash
$ source /opt/ros/noetic/setup.bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ catkin_make
```

Edit your ``~/.bashrc`` file so that the file ``catkin_ws/devel/setup.bash`` is sourced.

```bash
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

**You must perform the rest of the setup in a new terminal window.**  To make sure you have done the previous steps correctly, type ``$ roscd``.  You should be in the directory ``~/catkin_ws/devel``.

## Setup Your CompRobo GitHub Repository

First create a fork of <a-no-proxy href="https://github.com/comprobo20/comprobo20">the base CompRobo repository</a-no-proxy>.  If you don't know how to create a fork of a repository, check out [the GitHub documentation for forking a repository](https://docs.github.com/en/github/getting-started-with-github/fork-a-repo).  Next, clone your forked repository inside your catkin workspace.  You will also be performing the step of adding the CompRobo repository as a remote.

```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/your-user-name-here/comprobo20.git
$ cd comprobo20
$ git remote add upstream https://github.com/comprobo20/comprobo20.git
$ cd ~/catkin_ws
$ catkin_make
```

If ``catkin_make`` exits with no error messages, then your laptop is ready to use with the simulated Neato!  You may to close your current terminal as we have found that ``sourcing ~/catkin_ws/devel/setup.bash`` again seems to be required for everything to function properly.

## Melodic Python 3 Support

> Note: this is not needed if you are using ROS Noetic

Our goal will be to support both ROS Noetic and ROS Melodic *in Python 3*.  This means that in order to use Melodic, you should be setting your system up to work with Python 3 ROS code (Python 2.7 is the default in Melodic).

### Installing the ROS Client Packages for Python 3

Run the following commands to install the ROS Python client packages for Python 3.

```bash
$ sudo apt-get install python3-catkin-pkg-modules
$ sudo apt-get install python3-rospkg-modules
```

### Installing OpenCV

Make sure you have installed ``pip3``

```bash
$ sudo apt-get install python3-pip
```

Install ``Sckit-Build`` and ``OpenCV`` (we specify 4.2.0.34 to try to match the version that comes with Noetic)

```bash
$ pip3 install scikit-build
$ pip3 install opencv-python==4.2.0.34
```

## Setup on Zoom

In order to get Zoom working, perform the following steps.

```bash
$ sudo apt-get install wget
$ wget https://zoom.us/client/latest/zoom_amd64.deb
$ sudo apt install ./zoom_amd64.deb
``` 
