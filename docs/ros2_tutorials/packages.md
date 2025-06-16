# Packages

## Packages

When you create a node you need a package to put the node in. Multiple nodes can be put into one package. You already know this because you've typed the following commands:
``` bash
 ros2 run [package_name] [node_name]
 ros2 run turtlesim turtlesim_node
 ros2 run turtlesim turtle_teleop_key
```

Both the `turtlesim` and `teleop_turtle` nodes are in the `turtlesim` package.

The following command will create a package, and should be called when **within the src folder** of your workspace (/ros2_ws/src):

``` bash
 ros2 pkg create [package_name] --build-type [build_type]  --dependencies [dependecy 1] [dependency 2] [dependency...]
```

There are three build types: `cmake`, `ament_cmake`, and `ament_python`. If you don't assign one, `ament_cmake` will be chosen by default.

There could be several depend arguments. They might be as follows: "std_msgs", "rclpy", or "rclcpp". 

Once you are done creating a package, you can build the package by this command :
``` bash
 colcon build
```

When you call this command, make sure you are **within the root folder of your workspace** (/ros2_ws) instead of any other folders in the workspace, such as the src folder. Calling this command will build all the packages in the workspace, not just the ones most recently created.

If you want to build only one package, try:
```bash
 colcon build --packages-select [package_name]
```

As previously mentioned, when you are writing your code, you will need to build the package before any saved changes can take effect. 


## C++ Packages

Example: 
```bash
 ros2 pkg create example_cpp --build-type ament_cmake --dependencies rclcpp
```

A package should have 4 essential items within: `CMakeLists.txt`, `package.xml`, an `include` folder, and a `src` folder. Without them, the package would just be a regular folder.
```
CMakeLists.txt file that describes how to build the code within the package

include/<package_name> directory containing the public headers for the package

package.xml file containing meta information about the package

src directory containing the source code for the package
```

### CMakeLists.txt

Below is a simple example of a CmakeLists.txt.
``` CMake
cmake_minimum_required(VERSION 3.8)
project(example_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

The CMakeLists.txt that was auto-created when you called `ros2 pkg create` is basically the same thing as the above with a lot more unnecessary code that has been commented out. The following line is the most important and what you will need to change in order for your node to function.

``` CMake
add_executable(example_node src/example_node.cpp src/example.cpp)
```

The above line adds the `example_node.cpp` and `example.cpp` files, which you will create and add to the executables, so that they can be run. Both of the `.cpp` files will be placed in the src folder.

### src Folder

The source folder contains two `.cpp` files which hold the commands for the node. Nodes are often written using classes in c++. Conventionally, the `example_node.cpp` will hold the `int main()` function and the `example.cpp` will hold the class and function declarations. **For creating a publisher and a subscriber, here is also where the publisher.cpp and subscriber.cpp should be.**

### include Folder

The include folder will contain the `.hpp` files.

### package.xml file

The package.xml file containing meta information about the package



## Python Packages
Example:
```bash
 ros2 pkg create example_py  --build-type ament_python --dependencies rclpy
```

### package.xml file

The **package.xml** file contains meta information about the package.

### resource/<package_name> 

**resource/<package_name>** is the marker file for the package

### setup.cfg

**setup.cfg** is required when a package has executables, so `ros2 run` can find them.

### setup.py

**setup.py** contains instructions for how to install the package.

Example:
```
from setuptools import find_packages, setup

package_name = 'example_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maintainer_name',
    maintainer_email='maintainer_email@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
```

### my_package Folder

**package_name** - a directory with the same name as your package, used by ROS 2 tools to find your package containing **__init__.py**.

**For creating a publisher and a subscriber, this folder is also where the publisher.cpp and subscriber.cpp should be.**