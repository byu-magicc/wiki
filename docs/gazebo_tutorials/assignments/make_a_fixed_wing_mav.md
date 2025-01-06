# Making a Fixed Wing MAV

## Making a Fixed Wing MAV
In this assignment, we will build a fixed wing MAV. This is very similar to the previous assignment, except this time we will be copying the junker files in `rotor_gazebo` instead of the multirotor files.

The xacro file is particularly intimidating, mostly because of the large number of parameters required for the forces and moments plugin. Don't let this bother you. It's just a text file.

As an example, in the case you skipped the multirotor assignment, if I were building a new fixed wing MAV called "ares" the file structure would appear as follows, with the new files highlighted in bold, based on the junker equivalents. In the case of a fixed-wing. We don't need a yaml file for a ROS-based attitude controller like we do to calculate mixing in the multirotor.

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
|    |    |-- shredder.dae
|    |    |-- junker.dae
|    |    |-- ares.dae
|    |    |-- ...
|    |    |
|    +-- msg
|    +-- param
|    |-- urdf
|    |    +-- shredder
|    |    +-- hummingbird
|    |    |-- ares
|    |    |   |-- ares_base.xacro
|    |    |-- ...
|    |    |
|    +-- worlds
+-- fcu_sim_plugins
+-- sim_reset
```

## The Forces and Moments Plugin
Just like the multirotor, because Gazebo doesn't support aerodynamics, we apply a plugin to the MAV which takes the various states of the aircraft in the simulator and applies forces and moments to the parent_link. The forces and moments plugin is built following the method in the [UAV book](https://www.amazon.com/Small-Unmanned-Aircraft-Theory-Practice/dp/0691149216). We will talk about how that file is built in the next tutorial.

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
