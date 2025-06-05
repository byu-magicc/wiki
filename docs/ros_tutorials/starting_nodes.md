# Starting Nodes

## Prerequisites

For this tutorial we will use a lightweight simulator, please install it using:

```bash
 sudo apt-get install ros-kinetic-ros-tutorials
```



## roscore

`roscore` is the first thing you should run when using ROS.  You *must* have a roscore running in order for ROS nodes to communicate.

Please run:
```bash
 roscore
```

You will see something similar to:
```
... logging to~/.ros/log/9cf88ce4-b14d-11df-8a75-00251148e8cf/roslaunch-machine_name-13039.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://machine_name:33919/
ros_comm version 1.4.7

SUMMARY
========

PARAMETERS
* /rosversion
* /rosdistro

NODES

auto-starting new master
process[master]: started with pid [13054]
ROS_MASTER_URI=http://machine_name:11311/

setting /run_id to 9cf88ce4-b14d-11df-8a75-00251148e8cf
process[rosout-1]: started with pid [13067]
started core service [/rosout]
```

If `roscore` does not initialize, you probably have a network configuration issue. See the official ROS tutorials on
["Setup - Single Machine Configuration"](https://wiki.ros.org/ROS/NetworkSetup#Single_machine_configuration|Network).

If `roscore` does not initialize and sends a message about lack of permissions, probably the *~/.ros* folder is owned by *root*, change recursively the ownership of that folder with:

``` bash
 sudo chown -R <your_username> ~/.ros
```



## Starting Nodes

A node isn't really much more than an executable file within a ROS package.  We want to start the `turtlesim_node` in the `turtlesim` package.  In order to do so we will use `rosrun`.

`rosrun`  allows you to use the package name to directly run a node within a package (without having to know the package path).

Usage:
```bash
 rosrun [package_name] [node_name]
```



## Starting Turtlesim

We can now run the `turtlesim_node` in the `turtlesim` package.

Leave `roscore` running in one terminal and in a **new terminal** start `turtlesim_node` with:

```bash
 rosrun turtlesim turtlesim_node
```

You will see the turtlesim window:

![Turtlesim](figures/turtlesim.png)



## Naming Nodes

Often we wish to run multiple instances of the same node.  In a **new terminal** try running another `turtlesim_node`:

```bash
 rosrun turtlesim turtlesim_node
```

You'll notice ROS does not allow two nodes with the same name to be running at the same time.  We can remedy this problem by reassigning the name of the second node when we start it:

```bash
 rosrun turtlesim turtlesim_node __name:=leo
```

We can now start as many turtlesim nodes that we want as long as each has a unique name!



## Review

What was covered:

 * `roscore` = ros+core : master (provides name service for ROS) + rosout (stdout/stderr) + parameter server
 * `rosrun` = ros+run :  runs a node from a given package.
