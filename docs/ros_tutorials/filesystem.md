# The ROS Filesystem

Now that you have gotten a little experience with ROS you're ready to start working within the ROS file system.

## Quick Overview of Filesystem Concepts

### Packages
Packages are the software organization unit of ROS code. Each package can contain libraries, executables, scripts, or other artifacts.  The `turtlesim_node` and `turtle_teleop_key` nodes which you started previously are both contained within the `turtlesim` package. The `rqt_graph` node is contained inside the `rqt_graph` package.

### Manifest
A manifest is a description of a ''package''. Its serves to define dependencies between packages and to capture meta information about the package like version, maintainer, license, etc...

## Filesystem Tools
Code is spread across many ROS packages. Navigating with command-line tools such as `ls` and `cd` can be very tedious which is why ROS provides tools to help you.

### rospack
[rospack](https://wiki.ros.org/rospack) allows you to get information about packages. In this tutorial, we are only going to cover the `find` option, which returns the path to package.
Usage:

```
rospack find [package_name]
```

Example:

```
rospack find roscpp
```

Would return:
```
 YOUR_INSTALL_PATH/share/roscpp
```

If you installed ROS kinetic with `apt` on Ubuntu Linux you would see exactly: `/opt/ros/kinetic/share/roscpp`

### roscd
`roscd` is part of the [rosbash](https://wiki.ros.org/rosbash) suite. It allows you to change directory `cd` directly to a package or a stack.
Usage:
```
 roscd [locationname[/subdir]]
```
Run this example:
```
 roscd roscpp
```

To verify that we have changed to the `roscpp` package directory. Now let's print the working directory using the Unix command `pwd`:

You should see:

```
 YOUR_INSTALL_PATH/share/roscpp
```
You can see that `YOUR_INSTALL_PATH/share/roscpp` is the same path that `rospack find` gave in the previous example.
Note that `roscd`, like other ROS tools, will __only__ find ROS packages that are within the directories listed in your [ROS_PACKAGE_PATH](https://wiki.ros.org/ROS/EnvironmentVariables#ROS_PACKAGE_PATH).

To see what is in your `ROS_PACKAGE_PATH`
```
 echo $ROS_PACKAGE_PATH
```
Your `ROS_PACKAGE_PATH` should contain a list of directories where you have ROS packages separated by colons. Here is a typical example
```bash
 /opt/ros/kinetic/base/install/share:/opt/ros/kinetic/base/install/stacks
```

`roscd` can also move to a subdirectory of a package or stack. Try:
```
 roscd roscpp/cmake
 pwd
```
You should see:
```
 YOUR_INSTALL_PATH/share/roscpp/cmake
```

### roscd log
`roscd log` will take you to the folder where ROS stores log files. Note that if you have not run any ROS programs yet, this will yield an error saying that it does not yet exist.
If you have run some ROS program before, try:
`roscd log`

### rosls
[rosls](https://wiki.ros.org/rosbash#rosls) is also part of the `rosbash` suite. It allows you to `ls` directly in a package by name rather than by absolute path.
Usage:
```bash
 rosls [locationname[/subdir]]
```
Example:
```bash
 rosls roscpp_tutorials
```
Would return:
```
 cmake launch package.xml  srv
```

### Tab Completion
It can get tedious to type out an entire package name.  In the previous example, `roscpp_tutorials` is a fairly long name.  Luckily, some ROS tools support [TAB completion](https://en.wikipedia.org/wiki/Command_line_completion).
Start by typing:
```
 roscd roscpp_tut<<< now push the TAB key >>>
```
After pushing the `TAB` key, the command line should fill out the rest: `roscd roscpp_tutorials/`
This works because `roscpp_tutorials` is currently the only ROS package that starts with `roscpp_tut`.
Now try typing:
```
 roscd tur<<< now push the TAB key >>>
```

After pushing the `TAB` key, the command line should fill out as much as possible:
`
 roscd turtle
`
However, in this case there are multiple packages that begin with `turtle`. Try typing `TAB` another time.  This should display all the ROS packages that begin with `turtle`:
``` bash
 turtle_actionlib/  turtlesim/  turtle_tf/
```

On the command line you should still have:
```
 $ roscd turtle
```
Now type a `s` after `turtle` and then push `TAB`:
```
 roscd turtles<<< now push the TAB key >>>
```
Since there is only one package that start with `turtles`, you should see:
```
 roscd turtlesim/
```

## Review
You may have noticed a pattern with the naming of the ROS tools:

 * `rospack` = ros + pack(age)
 * `roscd` = ros + cd
 * `rosls` = ros + ls

This naming pattern holds for many of the ROS tools.
