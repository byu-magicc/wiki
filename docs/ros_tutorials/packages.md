# Packages

 ## Catkin Packages
When you create a new node you need to have a package to put it in. Multiple nodes can be put into one package. You already know this because you've typed the following commands:

``` bash
rosrun <package> <node>
rosrun turtlesim turtlesim_node
rosrun turtlesim turtle_teleop_key
```
Both the turtle_teleop_key node and turtlesim_node are in the turtlesim package.

So you will make a package and new node to put into it. The following command will create a package, and should be called when within the src folder within the catkin workspace :

``` bash
catkin_create_pkg <name of package> <depends1> <depends2> <depends3>
```

The depend arguments might be as follows : "std_msgs" "rospy" "roscpp"

Once you have made a package, you can build it by using the command :

``` bash
catkin_make
```

When you call this command make sure you are within the catkin workspace and not within any other folders in that workspace such as the src folder. Calling this command will build all packages located within the workspace, not just the ones most recently created. As previously mentioned, when you are writing your c++ code (probably within QtCreator) you will need to build the package that contains your node before any saved changes can take effect. Calling catkin_make will accomplish this, but it is cumbersome so it is convenient to build from within QtCreator using Ctrl-b. Both have the same effect.
Here is a link to shortcuts. [QtCreator Keyboard Shortcuts](https://wiki.qt.io/Qt_Creator_Keyboard_Shortcuts)


The package should have 4 essential items within: `CMakeLists.txt`, `package.xml`, an `include` folder and a `src` folder. These are what make a package a package. Without them, the package would just be a regular folder. You can ignore the `packages.xml` for now, but lets talk about the other 3.

## CMakeLists.txt
Below is a simple example of a CmakeLists.txt.
``` CMake
cmake_minimum_required(VERSION 2.8.3)
project(turtle_star)

find_package(catkin REQUIRED COMPONENTS
 std_msgs
 geometry_msgs
 roscpp
 rospy
)

catkin_package()

###########
## Build ##
###########
include_directories(include)
include_directories(
 ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(turtle_star
#   src/${PROJECT_NAME}/turtle_star.cpp
# )

## Declare a cpp executable
add_executable(example_node src/example_node.cpp src/example.cpp)

## Add cmake target dependencies of the executable/library
add_dependencies(turtle_star_node relative_nav_msgs_generate_messages_cpp)
target_link_libraries(turtle_star_node ${catkin_LIBRARIES})
```

The CMakeLists.txt that was auto created when you called catkin_create_package is basically the same thing as the above with a lot more unnecessary code that has been commented out. The following line is the most important and what you will need to change in order for your node to function.

 ``` CMake
add_executable(example_node src/example_node.cpp src/example.cpp)
```

The above line adds the `example_node.cpp` and `example.cpp` files, which you will create, to the executables so that they can be run. Both of the `.cpp` files will be placed in the src folder.

 ## src Folder
The source folder contains two `.cpp` files which hold the commands for the node. Nodes are often written using classes in c++. Conventionally, the `example_node.cpp` will hold the `int main()` function and the `example.cpp` will hold the class and function declarations.

 ## include Folder
The include folder will contain the `.h` files.
