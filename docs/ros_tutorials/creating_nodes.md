# Creating Nodes

Now that you know most everything you need to know about launch files it's time to learn how to create your own nodes.  However, before you do that, it'll be important for you to know how to set up a C++ environment, since that is the language we use most often.

## Compilation, Executables and Code
A quick word on how code works, and how the "toolchain" is set up for ROS.  Previously, you've simply typed the commands
 catkin_make
 rosrun <package> <node>
But what do these commands do?  Well, catkin is the official build system of ROS.  It's basically a customized version of CMake.  CMake generates "Makefiles" based on the rules given in the CMakeLists.txt file in each package.  If you ever go into the package subfolders of your build directory, you can look at the auto-generated makefiles, and realize why cmake is so helpful.  The makefile is then read by your compiler (in this case, g++) which reads the code and generates the machine code for your executables.  When you type
 rosrun <package> <node>
ROS goes to the executable, not your c++ files and loads what the compiler made.  This means that whenever you make a change to any c++ file, ''you have to go and re-run catkin_make'', otherwise the changes will not make it to the executables.

## Making Your Own Nodes
These links will take you to the official ROS tutorials which are pretty good at covering this subject.

### Creating Nodes in C++:

 [Writing a Simple Publisher and Subscriber (C++)](https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)

### Creating Nodes in Python:

[Writing a Simple Publisher and Subscriber (Python)](https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)

### Examining Your Nodes::

[Examining the Simple Publisher and Subscriber](https://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber)
