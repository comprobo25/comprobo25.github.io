---
title: "Coordinate Frames and Basic ROS Concepts"
toc_sticky: true
---

## Today
* [More Course Details](https://docs.google.com/presentation/d/19B0FtOo1qZqo8MxAklRULqWMokjUoezlMTHTjRkoUYE/edit?usp=sharing)
* Coordinate frames and coordinate transformations
* Questions / clarifications on basic ROS concepts
* Writing our first ROS node

## For Next Time
* Find a partner for the Warmup Project and get started (there is an intermediate deliverable due on class 4).


## What is a Robot Anyway?

While Neatos, Spot and manufacturing robotic arms seem to clearly be "robots", the line is not always so clear - e.g. when we talk about things like washing machines, Alexa, RC cars. We like to think about robots as something that has a "brain" (processor), one or more sensors, and one or more actuators. By this definition, yes, machine machines and Alexa, are in fact robots. However, <a-no-proxy href="https://roboticsandautomationnews.com/2020/05/11/opinion-what-is-a-robot-and-how-does-it-differ-from-a-machine/32192/">there are also (less academic) definitions</a-no-proxy> and shades of gray when what exactly is a robot. 

## Coordinate Frames and Coordinate Transforms in Robotics

> Likely you've encountered the notion of multiple coordinate systems before at some point in your academic career.  Depending on your path through Olin, you may already be familiar with the mechanics of how to map vectors between different coordinate systems (either in 2D or 3D).  In this exercise, you'll get a chance to refresh some of this knowledge and to also gain a conceptual understanding of how the notion of multiple coordinate systems plays out in robotics.


Before we begin, let's standardize our notation.

$$\begin{eqnarray}
\mathbf{p}_{/W} &\triangleq& \mbox{a point, p, expressed in coordinate system W} \\
\mathbf{p}_{/N} &\triangleq& \mbox{a point, p, expressed in coordinate system N} \\
\hat{\mathbf{i}}_{N} &\triangleq& \mbox{a unit vector in the i-hat direction of coordinate system N} \\
\hat{\mathbf{j}}_{N} &\triangleq& \mbox{a unit vector in the j-hat direction of coordinate system N} \\
\hat{\mathbf{r}}_{W\rightarrow N} &\triangleq& \mbox{a vector pointing from the origin of W to the origin of N} \\
\mathbf{r}_{W \rightarrow N / W} &\triangleq& \hat{\mathbf{r}}_{W\rightarrow N}\mbox{ expressed in coordinate system W} \\
\hat{\mathbf{i}}_{N/W} &\triangleq& \hat{\mathbf{i}}_{N}\mbox{ expressed in coordinate system W} \\
\hat{\mathbf{j}}_{N/W} &\triangleq& \hat{\mathbf{j}}_{N}\mbox{ expressed in coordinate system W}\end{eqnarray}$$

Suppose your Neato is at position 3.0m, 5.0m with a heading of 30 degrees (where counter-clockwise rotation is positive) in a coordinate system called ``world``.  Draw a picture.  Make sure to label the axes of the ``world`` coordinate system (don't worry about the z-axis).

In robotics, we frequently need to express the position of various entities (e.g., obstacles, goal locations, other robots, walls, doorways, etc.).  While we could express all of these positions in terms of the coordinate system ``world``, in many situations this will be cumbersome.

**Exercise:** Taking the Neato as an example, make a list of the coordinate systems that you feel would be convenient to define.  For each coordinate system, define its origin and give a few examples of entities that would be natural to express in the coordinate system. 

### ``base_link``

Next, we'll define ``base_link``, which will serve as our robot-centric coordinate system.  The origin of this coordinate system will be at the midpoint of the line connecting the robot's wheels.  The x-axis will point forwards, the y-axis will point to the left, and the z-axis will point up.  Update your drawing to indicate the position of the ``base_link`` coordinate axes (again, don't worry about the z-axis).

Now that we have defined our new coordinate system, we'd like to be able to take points expressed in this coordinate system and map them to the ``world`` coordinate system (and vice-versa).  In order to do this, we need to specify the relationship between these two coordinate systems.  A natural way to specify the relationship between two coordinate systems is to specify the position of the origin of one coordinate system in the other as well as the directions of the coordinate axes of one frame in the other.  Going back to our original example we can say that the coordinate axes of the Neato's ``base_link`` coordinate system are at position 3.0m, 5.0m with a rotation of 30 degrees relative to the coordinate axes of the ``world`` coordinate frame.  We usually think of this information as defining the transformation from ``world`` to ``base_link``.  It turns out that with just this information, we can map vectors between these two coordinate systems.  ROS has robust infrastructure to handle these transformations automatically, so for the most part when writing ROS code, you don't have to worry about how to actually perform these transformations.  However, to build your understanding, we'll dig into this the details a bit.

### From ``base_link`` to ``world``

**Exercise:** Determine the coordinates of a point located at (1.0m, 0.0m) in the ``base_link`` coordinate system in the ``world`` coordinate system.  First draw the point on the board to make sure everyone agrees what its location is.  Once you've determined your answer, how can you tell if you are right?

**Exercise:** Determine the coordinates of a point located at (0.0m, 1.0m) in the ``base_link`` coordinate system in the ``world`` coordinate system.  First draw the point on the board to make sure everyone agrees what its location is.  Once you've determined your answer, how can you tell if you are right?


**Exercise:**  Determine the coordinates of a point located at (x, y) in the ``base_link`` coordinate system in the ``world`` coordinate system.  If you are having trouble operationalizing your answer in terms of equations, you can define it in terms of high-level operations (e.g., translations, rotations, etc.).


### From ``world`` to ``base_link``

There are multiple ways to tackle this one.  We think it's easiest to do algebraically, but you can do it in terms of geometry / trigonometry too.  Don't get too hung up on the mechanics, try to understand conceptually how you would solve the problem.

**Exercise:** Determine the coordinates of a point located at (0.0m, 1.0m) in the ``world`` coordinate system in the ``base_link`` coordinate system.  First draw the point on the board to make sure everyone agrees what its location is.  Once you've determined your answer, how can you tell if you are right?


**Exercise:** Determine the coordinates of a point located at (1.0m, 0.0m) in the ``world`` coordinate system in the ``base_link`` coordinate system.  First draw the point on the board to make sure everyone agrees what its location is.  Once you've determined your answer, how can you tell if you are right?

**Exercise:**  Determine the coordinates of a point located at (x, y) in the ``world`` coordinate system in the ``base_link`` coordinate system.  If you are having trouble operationalizing your answer in terms of equations, you can define it in terms of high-level operations (e.g., translations, rotations, etc.).


### Static Versus Dynamic Coordinate Transformations

The relationship between some coordinate systems are dynamic (meaning they change over time) and some are static (meaning they are constant over time).

**Exercise:**  Assume that our Neato robot can move about in the ``world`` by using its wheels.  Is the relationship between ``world`` and ``base_link`` static or dynamic?  Given the coordinate systems you came up with earlier, list some examples of coordinate system relationships that are static and some that are dynamic.
Before Starting


## Coding Exercises

Sample solutions for these exercises can be found in the [class_activities_and_resources Github repo](https://github.com/comprobo22/class_activities_and_resources).

### Creating a ROS package

Let's write our code today in a package called ``in_class_day02``

```bash
$ cd ~/ros2_ws/src
$ ros2 pkg create in_class_day02 --build-type ament_python --node-name send_message --dependencies rclpy std_msgs geometry_msgs sensor_msgs
```

This command will create the package for you and also a node called ``send_message`` that should be located in the following location:

```bash
~/ros2_ws/src/in_class_day02/in_class_day02/send_message.py
```

By default it will look like this:

```python
def main():
    print('Hi from in_class_day02.')


if __name__ == '__main__':
    main()
````

### Building and Running your Node

You can build your node by running:

```bash
$ cd ~/ros2_ws
$ colcon build --symlink-install
```

What does this mysterious ``--symlink-install`` do?  Well, a [symlink](https://en.wikipedia.org/wiki/Symbolic_link) is a special type of file that points to another file.  In this case we will have a special file in our ``install`` directory that points to our Python script in our ``src`` directory.  In this way, we can modify the Python script ``send_message.py`` and run the modified ROS node without constantly running ``colcon build``.  To see the symlink run the following command.

```bash
$ ls -l ~/ros2_ws/build/in_class_day02/in_class_day02

lrwxrwxrwx 1 parallels parallels 108 Sep  5 20:14 /home/parallels/ros2_ws/build/in_class_day02/in_class_day02 -> /home/parallels/ros2_ws/src/class_activities_and_resources/in_class_day02/in_class_day02
```
Notice how the ``->`` sign indicates a pointer (symlink) from the ``build`` directory back to the ``src`` directory.

In order to run the node, first source the ``install.bash`` script and then use ``ros2 run`` (Hint: try using tab completion when typing in the package and node names)
```bash
$ source ~/ros2_ws/install/setup.bash
$ ros2 run in_class_day02 send_message
```

### Creating a Skeleton ROS Node

In order for your Python program to interface with ROS, you have to call the appropriate functions.  The easiest way to do this is by creating a subclass of the ``rclpy.node.Node`` class.  If you are a bit rusty on your Python object-oriented concepts, take a look back at your notes from SoftDes (also let us know if you have a favorite resource for this).  Taking the code that was automatically created in the ``ros2 pkg`` step and converting it into a ROS node, would look like this.

```python
""" This script explores publishing ROS messages in ROS using Python """
import rclpy
from rclpy.node import Node

class SendMessageNode(Node):
    def __init__(self):
        super().__init__('send_message_node')
        # Create a timer that fires ten times per second
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)

    def run_loop(self):
        print('Hi from in_class_day02.')

def main(args=None):
    rclpy.init(args=args)      # Initialize communication with ROS
    node = SendMessageNode()   # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    rclpy.shutdown()           # cleanup

if __name__ == '__main__':
    main()
```

As before, you can run the node using ``ros2 run``

```bash
$ ros2 run in_class_day02 send_message
```

You will now have to use ``ctrl-c`` to stop execution of your node.  You should see the string ``Hi from in_class_day02`` print repeatedly onto the console.

### Creating ROS Messages in a Python Program (walkthrough in MAC126)

ROS messages are represented in Python as objects.  In order to create a ROS message you must call the ``__init__`` method for the ROS message class.  As an example, suppose we want to create a ROS message of type ``geometry_msgs/msg/PointStamped``.  The first thing we need to do is import the Python module that defines the ``PointStamped`` class.  The message type ``geometry_msgs/msg/PointStamped`` indicates that the ``PointStamped`` message type is part of the ``geometry_msgs`` package.  All of the definitions for messages stored in the ``geometry_msgs`` package will be in a sub-package called ``geometry_msgs.msg``.  In order to import the correct class definition into our Python code, we can create a new Python script at ``~/ros2_ws/src/in_class_day02/in_class_day02/send_message.py`` and add the following line to the top of our ``send_message.py`` script.

```python
from geometry_msgs.msg import PointStamped
```

Now we will want to create a message of type PointStamped.  In order to do this, we must determine what attributes the PointStamped object contains.  In order to do this, run

```bash
$ ros2 interface show geometry_msgs/msg/PointStamped
# This represents a Point with reference coordinate frame and timestamp

std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
Point point
	float64 x
	float64 y
	float64 z

std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Point point
  float64 x
  float64 y
  float64 z
```

If we look at the lines that are unindented (aligned all the way to the left), we will see the attributes that comprise a ``PointStamped`` object.  These attributes are header (which is of type ``std_msgs/msg/Header``) and point (which is of type ``geometry_msgs/msg/Point``).  The indented lines define the definition of the ``std_msgs/msg/Header`` and ``geometry_msgs/msg/Point`` messages.  To see this, try doing running ``$ ros2 interface show`` for both ``std_msgs/msg/Header`` and ``geometry_msgs/msg/Point``.

In order to create the PointStamped object, we will have to specify both a ``std_msgs/msg/Header`` and a ``geometry_msgs/msg/Point``.  Based on the definitions of these two types given by ``$ ros2 interface show`` (output omitted, but you can see it in a slightly different form above), we know that for the ``std_msgs/msg/Header`` message we need to specify seq, stamp, and frame_id. It will turn out that we don't have to worry about the ``seq`` (it will automatically be filled out by the ROS runtime when we publish our message), the stamp field is a ROS time object (see this tutorial), and the ``frame_id`` field is simply the name of the coordinate frame (more on coordinate frames later) in which the point is defined.  Likewise, the ``geometry_msgs/msg/Point`` object needs three floating point values representing the $$x$$, $$y$$, and $$z$$ coordinates of a point.  We can create these two messages using the standard method of creating objects in Python.  In this example we will be using the keyword arguments form of calling a Python function which will make your code a bit more robust and a lot more readable.  First, we add the relevant import statements:

```python
from std_msgs.msg import Header
from geometry_msgs.msg import Point
```

Now we can define the header and point that will eventually compose our ``PointStamped`` message.  Let's put this code in the ``run_loop`` function so we can publish the message each time ``run_loop`` is called.

```python
my_header = Header(stamp=self.get_clock().now().to_msg(), frame_id="odom")
my_point = Point(x=1.0, y=2.0, z=0.0)
```

Now that we have the two fields required for our PointStamped message, we can go ahead and create it.

```python
my_point_stamped = PointStamped(header=my_header, point=my_point)
```

To see what our resultant message looks like, we can print it out:

```python
print(my_point_stamped)
```

This will produce the output:

```bash
geometry_msgs.msg.PointStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1662424018, nanosec=755100091), frame_id='odom'), point=geometry_msgs.msg.Point(x=1.0, y=2.0, z=0.0))
```

> Note that instead of creating the two attributes of PointStamped in separate lines, we can do everything in one line as:
```python
my_point_stamped = PointStamped(header=Header(stamp=self.get_clock().now().to_msg(),
                                              frame_id="odom"),
                                point=Point(x=1.0, y=2.0, z=0.0))
```

In order to do something interesting, let's publish our message to a topic called ``/my_point``

First, we create the publisher in our ``__init__`` method by adding the following line to the end of that function.
```python
self.publisher = self.create_publisher(PointStamped, 'my_point', 10)
```

You can publish ``my_point_stamped`` by adding the following code to the end of your ``run_loop`` function

```python
self.publisher.publish(my_point_stamped)
```

Try running your code!

How can you be sure whether it is working or not?  Try visualizing the results in rviz.  What steps are needed to make this work?


### Callbacks (walkthrough in main room)

[Callback functions](https://en.wikipedia.org/wiki/Callback_(computer_programming)) are a fundamental concept in ROS (and we just used them to create our timer whether we knew it or not).  Specifically, they are used to process incoming messages inside a ROS node once we have subscribed to a particular topic.  Let's write some code to listen to the message we created in the previous step.

First, let's create a new ROS node in a file called ``receive_message.py`` in the directory ``~/ros2_ws/src/in_class_day02/in_class_day02``.  We'll start out with the standard first line as well as a header comment, import the correct message type, and initialize our ROS node:

```python
""" Investigate receiving a message using a callback function """
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

class ReceiveMessageNode(Node):
    def __init__(self):
        super().__init__('receive_message_node')

def main(args=None):
    rclpy.init(args=args)         # Initialize communication with ROS
    node = ReceiveMessageNode()   # Create our Node
    rclpy.spin(node)              # Run the Node until ready to shutdown
    rclpy.shutdown()              # cleanup

if __name__ == '__main__':
    main()
```

In order to run the node, we have to add it to our ``setup.py`` file, which is located in ``~/ros2_ws/src/in_class_day02/setup.py``.  We can modify the file as follows.

```python
    entry_points={
        'console_scripts': [
            'send_message = in_class_day02_solutions.send_message:main',
            'receive_message = in_class_day02_solutions.receive_message:main'
        ],
    },
```

Once you've modified ``setup.py``, you'll need to do another ``colcon build``.

```bash
$ cd ~/ros2_ws
$ colcon build --symlink-install
```

You can now run your node using the command:
```bash
$ ros2 run in_class_day02 receive_message
```

As of now, it won't do anything.

Next, we will define our callback function.  Our callback function takes as input a single parameter which will be a Python object of the type that is being published on the topic that we subscribe to.  Eventually we will be subscribing to the topic ``/my_point`` which means that we will be writing a callback function that handles objects of type ``geometry_msgs/PointStamped``.  As a test, let's make our callback function simply print out the header attribute of the PointStamped message.  This function should be added as a method of our ``ReceiveMessageNode`` class.

```python
def process_point(self, msg):
    print(msg.header)
```

Next, we must subscribe to the appropriate topic by adding this line to the ``__init__`` method.

```python
self.sub = self.create_subscription(PointStamped, 'my_point', self.process_point, 10)
```

The ROS runtime will take care of calling our ``process_point`` function whenever a new message is ready!  There is nothing more we have to do to make this happen!rospy.spin()

Go ahead and run your ROS node ``receive_message`` and see what happens!  (make sure your ``send_message`` node is also running)

### Viewing the Results in RViz

We can open up rviz and visualize the message.  First, open rviz.

```bash
$ rviz2
```

Next, click ``add``, ``by topic``, and select your marker message.  Make sure to set the ``fixed frame`` appropriately.
