# Additional Launch File Syntax

## Topic Remapping in Launch Files

When we first learned about launch files we had to do some topic remapping in order to get mimic to work properly. The code is repeated here:

(Python)
``` python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )
    ])
```


(XML)
``` XML
<launch>
  <node pkg="turtlesim" exec="turtlesim_node" name="sim" namespace="turtlesim1"/>
  <node pkg="turtlesim" exec="turtlesim_node" name="sim" namespace="turtlesim2"/>
  <node pkg="turtlesim" exec="mimic" name="mimic">
    <remap from="/input/pose" to="/turtlesim1/turtle1/pose"/>
    <remap from="/output/cmd_vel" to="/turtlesim2/turtle1/cmd_vel"/>
  </node>
</launch>
```

The syntax above can be used whenever topic remapping is required within launch files. Notice that the syntax above places the remapping inside the node `mimic`:

(Python)
``` python
    Node(
        package='turtlesim',
        executable='mimic',
        name='mimic',
        remappings=[
            ('/input/pose', '/turtlesim1/turtle1/pose'),
            ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
        ]
    )
```

(XML)
``` XML
  <node pkg="turtlesim" exec="mimic" name="mimic">
    <remap from="/input/pose" to="/turtlesim1/turtle1/pose"/>
    <remap from="/output/cmd_vel" to="/turtlesim2/turtle1/cmd_vel"/>
```

The syntax below is also used often but don't be confused because in this case the remapping is done outside of the node argument. When using this syntax, the remapping must be called before the node it acts on, or else it won't take effect. This is true for remapping topics and node names.

(Python)
``` python
    SetRemap(src='/input/pose', dst='/turtlesim1/turtle1/pose'),
    SetRemap(src='/output/cmd_vel', dst='/turtlesim2/turtle1/cmd_vel'),
    Node(
        package='turtlesim',
        executable='mimic',
        name='mimic'
    )
```

(XML)
``` xml
    <remap from="/george/turtle1/cmd_vel" to="/turtle1/cmd_vel"/>

    <group ns="george">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
    </group>
```



## Loading Parameters in Launch Files

See ["2 Parameters"](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html#parameters) in ["Managing large projects"](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html#managing-large-projects).

### YAML Files Example

Previously using `ros2 param dump` we created a YAML file which stored parameter values for us.  We can also create this file manually.  We just need to create a file with the .yaml extension which holds all our parameter information.  An example is the following which is contained within ps2.yaml:


``` yaml
joy_commander: {
 x_axis: 3,
 y_axis: 2,
 yaw_axis: 0,

 alt_axis: 5,

 x_sign: 1,
 y_sign: -1,
 yaw_sign: -1,

 mode_but: 0,
 llpp_but: 1,
 }
```

This set of parameters defines buttons on a joystick controller for use with the joy_commander node.  Since these parameters change with the type of controller used (Xbox 360 vs. PS2) it makes sense to group them within a .yaml file.

In order to load a .yaml file within a launch file the following syntax is used:

``` xml
 <param from="$(find rosparam)/example.yaml"/>
```

For additional information about launch file formats, consult the documentation [Using Python, XML, and YAML for ROS 2 Launch Files](https://docs.ros.org/en/jazzy/How-To-Guides/Launch-file-different-formats.html) and [Migrating launch files from ROS 1 to ROS 2](https://docs.ros.org/en/galactic/How-To-Guides/Launch-files-migration-guide.html).
