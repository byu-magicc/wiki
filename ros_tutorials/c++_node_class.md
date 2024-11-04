# Nodes Based on Classes (C++)


# Classes

If you have taken ME 235 and/or CS 142, you'll remember C++ classes.  If you haven't taken either of those, or you have and you're totally lost, then you should probably brush up on your C++.  [This](https://www.cplusplus.com/doc/tutorial/) is a pretty good tutorial that should help you get up to speed on C++ programming.

In short, we use classes because they are convenient.  They may not seem so convenient at first, but being able to split up your code into different files makes it a lot easier, and classes make it easy to store variables in between function calls, and limit the scope of functions.  You could just program every node in one giant main cpp file, but that eventually becomes unreasonable.

In the MAGICC lab, we generally organize our nodes as follows:

![node_example.png](figures/node_example.png)

We have the `CMakelists.txt` file that you're used to seeing, the `package.xml`, but then we make an include folder with the `.h` file inside, the `.cpp` file that implements the class functions, and then the main file, which is more of a ROS daemon than anything (spins up the node, and connects it to ROS).  Almost all of the actual work is done in the class `.cpp` file.

# Example: Moving Sensor Package

The picture above has the general outline of what is in each node, but I've created an example node which listens to the turtlesim pose message and publishes a boolean flag of whether or not it is moving.  I called my package the "moving_sensor" package.

This package is in a git repo [here](https://gitlab.magiccvs.byu.edu/lab/moving_sensor) If you want, you can download or clone the package and put it in your catkin_ws/src folder.

The file structure appears as follows:

```
moving_sensor
├── CMakeLists.txt
├── package.xml
├── include
│   ├── moving_sensor
│   │   └── moving_sensor.h
├── src
│   ├── moving_sensor.cpp
│   └── moving_sensor_node.cpp
```

### CMakeLists.txt
Let's look at the `CMakeLists.txt` of this package.  I've highlighted what each line does in this file below

``` CMake
cmake_minimum_required(VERSION 2.8.3)
project(moving_sensor)  # <-- this is the project name, should match in the package.xml

find_package(catkin REQUIRED COMPONENTS	# This goes out and finds the packages required
  turtlesim                             # for our moving sensor.  We need turtlesim because
  std_msgs                              # we are listening to turtlesim/Pose messages and we
  roscpp                                # need std_msgs because we are publishing a std_msgs/Bool
  rospy                                 # Often, we need Eigen, tf, and other packages that we use in
)                                       # the node

catkin_package() # This declares our node as a catkin package, so it gets picked up by a catkin_make

include_directories(include) # This line makes sure the compiler looks in the include folder for our headers
include_directories(
  ${catkin_INCLUDE_DIRS}     # This line adds in the headers already added by catkin (ros/ros.h)
)

## Declare a cpp executable 		# These lines create the executable (node).  You can make more than
add_executable(moving_sensor_node 	# one node per package, each node should have only one main() function
	src/moving_sensor_node.cpp      # We explicitly tell it which .cpp files to compile into that node
	src/moving_sensor.cpp
)

add_dependencies(moving_sensor_node  # This line tells catkin to first build the messages in turtlesim
	turtlesim_generate_messages_cpp  # before trying to build this node.  That is because we rely on the
)                                    # turtlesim/Pose message, which becomes a header file after it's compiled

target_link_libraries(moving_sensor_node # This links up the node with the compiled libraries catkin knows about
  ${catkin_LIBRARIES}                    # Basically, it allows the node to leverage ROS libraries.
)
```

### package.xml
Let's look now at the `package.xml`.  This file is by far the easiest of the bunch it's pretty much self-explanatory

``` xml
<?xml version="1.0"?>
<package>
  <name>moving_sensor</name>
  <version>0.0.0</version>
  <description>The moving_sensor package</description>

  <maintainer email="superjax08@gmail.com">James Jackson</maintainer>

  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>turtlesim</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>

  <run_depend>turtlesim</run_depend>
  <run_depend>std_msgs</run_depend>
  <run_depend>roscpp</run_depend>
  <run_depend>rospy</run_depend>

</package>
```

### moving_sensor.h
``` C++
#ifndef MOVING_SENSOR_H
#define MOVING_SENSOR_H

#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <std_msgs/Bool.h>

namespace moving_sensor
{

class movingSensor
{

public:

  movingSensor();

private:

  //***************** NODE HANDLES ***************//
  ros::NodeHandle nh_;         // public node handle for subscribing, publishing, etc.
  ros::NodeHandle nh_private_; // private node handle for pulling parameter values from the parameter server

  //***************** PUBLISHERS AND SUBSCRIBERS ***************//
  ros::Subscriber pose_subscriber_;
  // will end up getting hooked up to the callback for the Pose message

  ros::Publisher bool_publisher_;
  // will publish the flag to ROS

  //***************** PARAMETERS ***************//
  double threshold_;
  // a parameter we get from the ROS server, in this case the value below which
  // we consider the turtle as not moving.  This is basically a class variable at this point,
  // but it is distinct from the other class variables, so we separate them here.

  //***************** STATE VARIABLES ***************//
  // in this node, we don't have any variables.  Often though, we need to remember
  // things between loops, so we could create variables here to hold those values

  //***************** CALLBACKS ***************//
  void poseCallback(const turtlesim::PoseConstPtr &msg);
  // this function will get called every time ROS "spins"
  // and there is a Pose message in the queue.  More on this
  // later

  //***************** FUNCTIONS ***************//
  // Also, in this node, we don't have any "helper" functions.  These are useful
  // if you need to break up the work in the node into different functions
};

} // namespace moving_sensor

#endif // movingSensor_H
```

### moving_sensor.cpp
``` C++
#include "moving_sensor/moving_sensor.h"

namespace moving_sensor
{

movingSensor::movingSensor() :
  nh_(ros::NodeHandle()),             /* This is an initialization list */
  nh_private_(ros::NodeHandle("~"))   /* if you don't know what that means, go read about it */
{
  //***************** RETREIVE PARAMS ***************//
  nh_private_.param<double>("threshold", threshold_, 0.0001);
  // This will pull the "threshold" parameter from the ROS server, and store it in the threshold_ variable.
  // If no value is specified on the ROS param server, then the default value of 0.0001 will be applied

  //***************** NODE HANDLES ***************//
  pose_subscriber_ = nh_.subscribe("turtle1/pose", 1, &movingSensor::poseCallback, this);
  // This connects the poseCallback function with the reception of a Pose message on the "turtle1/pose" topic
  // ROS will essentially call the poseCallback function every time it receives a message on that topic.
  // the "1" indicates the length of the queue to hold messages before tossing them.  In this case, our callback
  // function is so fast that 1 is sufficient.

  bool_publisher_ = nh_.advertise<std_msgs::Bool>("is_moving", 1);
  // This connects a std_msgs::Bool message on the "is_moving" topic.  The 1 also indicates the length of the queue
  // before tossing messages.  Publishers are generally so fast that 1 almost always works.
}

void movingSensor::poseCallback(const turtlesim::PoseConstPtr &msg)
// This function runs every time we get a turtlesim::Pose message on the "turtle1/pose" topic.
// We generally use the const <message>ConstPtr &msg syntax to prevent our node from accidentally
// changing the message, in the case that another node is also listening to it.
{
  std_msgs::Bool out_flag;  // create a new message to store the result of our check in
  if(msg->linear_velocity > threshold_){   // figure out if our velocity is more than the threshold
    out_flag.data = true; // save the result in our new message
  }else{
    out_flag.data = false;
  }

  // publish the message to ROS
  bool_publisher_.publish(out_flag);
}

} // namespace moving_sensor
```

### moving_sensor_node.cpp
``` C++
#include <ros/ros.h>
#include "moving_sensor/moving_sensor.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "moving_sensor_node"); // register the node on ROS
  ros::NodeHandle nh; // get a handle to ROS

  moving_sensor::movingSensor Thing;  // instatiate our class object

  ros::spin(); // check for new messages and call the callback if we get one

  return 0;
}
```
