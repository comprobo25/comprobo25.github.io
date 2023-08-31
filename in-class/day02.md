---
title: "Coordinate Frames and Basic ROS Concepts"
toc_sticky: true
---

## Today
* [More Course Details](https://docs.google.com/presentation/d/1GNKZft9K8El-Fa7mfzyOZ7l036KAhCNJzNK4WZGQ_x4/edit?usp=sharing)
* Questions / clarifications on basic ROS concepts
* Writing our first ROS node

## For Next Time
* Find a partner for the Warmup Project and get started (there is an intermediate deliverable that we would like you to aim for by class 4).


## Coding Exercises

Sample solutions for these exercises can be found in the [class_activities_and_resources Github repo](https://github.com/comprobo23/class_activities_and_resources).  If you'd like to organize your work under GitHub repo, we suggest you create one now to hold the various ROS packages you will be creating.  Alternatively, you can fork the repo ``class_activities_and_resources``.

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
