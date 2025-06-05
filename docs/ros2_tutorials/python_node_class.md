# Nodes Based On Classes (Python)



## Using Python

Python is an awesome language for quickly prototyping.  C++ is way faster than python, but in my experience, I can throw together a python node in about half the time it would take for me to put a C++ node together.  Debugging python is also a lot easier and faster, if only because you don't have to compile between code changes and you have a terminal to try code during a breakpoint.  Often, I'll create a prototype in python, and once it's working, I'll convert it to C++ to get a massive speedup.  I find that this workflow is typically more efficient than trying to do my debugging in C++.

You don't have to just use Python.  There are some awesome libraries that allow you to merge C++ and python into a single node.  This is a more advanced topic, and will probably be covered in later tutorials.  Obviously, you can have both python and C++ in the same package.


## ROS2 Package organization

Because python doesn't actually have to be compiled, you don't technically need a `CMakeLists.txt`.  You do need a `package.xml`, though, so `ros2 pkg` can recognize your package for what it is.  If you want to install your python node, then you will need to use the CMakeLists.txt, so that colcon can move your python files to the install directories.

## Classes

In short, we use classes because they are convenient. They may not seem so convenient at first, but being able to split up your code into different files makes it a lot easier, and classes make it easy to store variables in between function calls, and limit the scope of functions. You could just program every node in one giant main script, but that eventually becomes unreasonable.

In the MAGICC lab, we generally organize our nodes as follows:

![Python Node Example](figures/node_class/node_class_example_py.png)

The  `__init__.py` file (which is empty) tells python that your node is a python module.  This is also the reason for putting our scripts in a subfolder of the src folder named the same as our package.  Python is pretty flexible, and you can actually put your files wherever you want, but the above structure is the officially recommended organization.


## Example: Moving Sensor Package

The file structure appears as follows:

```
moving_sensor
├── package.xml
├── setup.py
├── setup.cfg
├── resource
│   ├── moving_sensor
├── (test)
├── moving_sensor
│   ├── __init__.py
│   ├── moving_sensor.py
```


### moving_sensor.py

``` python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from bool_interfaces.msg import Bool

class MovingSensor(Node):
    def __init__(self):
        super().__init__('moving_sensor')
        
        self.pose_subscriber = self.create_subscription(
            Pose,
            'pose',
            self.pose_callback,
            1
        )

        self.is_moving_publisher = self.create_publisher(
            Bool,
            'is_moving',
            1
        )

        self.declare_parameter('threshold', 0.0)
        self.threshold = self.get_parameter('threshold').get_parameter_value().double_value

    def pose_callback(self, msg):
        out_msg = Bool()
        if msg.linear_velocity > self.threshold:
            out_msg.data = True
        else:
            out_msg.data = False

        self.is_moving_publisher.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MovingSensor()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```
