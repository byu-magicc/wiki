# Making a Multirotor

## Making a Multirotor
For this assignment, you are to make your own multirotor. It doesn't need to fly, but you should be able to load your model into gazebo and see it.

If I were making a new MAV, I would duplicate either the hummingbird or the shredder files and rename them, then make the necessary changes. This is by far the easiest way to build a multirotor. For example, if I were making a new multirotor called "titan" the new file structure would look as follows, with new files bolded, based on the hummingbird and shredder equivalents.

```
fcu_sim
+-- attitude_controller
+-- joy_sim
+-- fcu_sim
|    |-- CMakeLists.txt
|    |-- package.xml
|    +-- cmake
|    +-- include
|    |-- launch
|    |    |-- spawn_mav.launch
|    |    |-- test.launch
|    |    |-- ...
|    |    |
|    |-- meshes
|    |    |-- firefly.dae
|    |    |-- hummingbird.dae
|    |    |-- titan.dae
|    |    |-- ...
|    |    |
|    +-- msg
|    |-- param
|    |    |-- hummingbird.yaml
|    |    |-- shredder.yaml
|    |    |-- titan.yaml
|    |    |-- ...
|    |    |
|    |-- urdf
|    |    +-- shredder
|    |    +-- hummingbird
|    |    |-- titan
|    |    |   |-- titan_base.xacro
|    |    |-- ...
|    |    |
|    +-- worlds
+-- fcu_sim_plugins
+-- sim_reset
```

## YAML File
We haven't yet talked about the YAML file. For now, the YAML file describes the same physical constants in the .xacro file. As it currently stands, the YAML file is used by ROS to correctly implement the attitude controller whereas the xacro file is used by gazebo to define the dynamics. In future iterations of the rotor_gazebo package, these could maybe be combined into a single YAML file, but that is complicated to get right. Until then, you'll need to make sure that the two match. It's a little confusing to have the two files, but it's necessary to ensure that both ROS and Gazebo know how the MAV is configured. (An [example](https://answers.ros.org/question/115226/how-to-pass-parameters-loaded-from-a-yaml-file-into-a-xacro/) of how this could be done.)

## Xacro File Features
Another thing to note is that because gazebo doesn't understand aerodynamics natively, there is a "multirotor_base_plugin" included in the relevant xacro. This plugin applies forces and torques to its parent_link based on motor speeds. The rotors are also connected to a plugin which are instantiated by the "vertical rotor" xacro. They model the propellers speeding up and slowing down as a first-order system based on the time_constant arguments. After building your model, you may need to adjust these constants to better reflect the performance of your MAV.

## Launch File
To help you a little more, here is a sample `test.launch` that you might use to launch your MAV. If your file structure is correct, this will launch gazebo and put your new MAV in the gazebo world.
```xml
<launch>
  <arg name="mav_name"            default="titan"/>

  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find fcu_sim)/worlds/basic.world"/>
    <arg name="paused" value="false"/>
    <arg name="gui" value="true"/>
  </include>

  <!-- Spawn MAV -->
  <include file="$(find rotor_gazebo)/launch/spawn_mav.launch">
    <arg name="mav_name"            value="$(arg mav_name)" />
    <arg name="model"               value="$(find fcu_sim)/urdf/$(arg mav_name)/$(arg mav_name)_base.xacro" />
  </include>
</launch>
```
