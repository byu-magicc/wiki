# Make a Robot

## Make a Robot
In this tutorial, we will make ourselves our own robot. We will be using an xacro file, rather than a straight SDF as described in the official Gazebo Tutorials. xacro is read by Gazebo, then converted to SDF, and it uses far less typing to create an xacro. (xacro means "xml-macro") which allows us to leverage parts of other files within individual xacro files. You'll find this very useful later.

Lets make the same robot you make in the official [gazebo tutorials](https://gazebosim.org/tutorials?tut=build_robot&cat=build_robot), starting with a blank file named `pioneer_base.xacro`

### Starting the File
The first thing you'll do is define the the xml version and the fact you're building a robot.
```xml
<?xml version="1.0"?>

<robot name="pioneer" xmlns:xacro="https://wiki.ros.org/xacro">
</robot>
```

### Setting Parameters
Next, let's define some parameters, like the robot mass, length, width, height, wheel radius, inertia, and so on...
```xml
<?xml version="1.0"?>

<robot name="pioneer" xmlns:xacro="https://wiki.ros.org/xacro">
  <xacro:property name="mass" value="0.5755" />
  <xacro:property name="width" value="0.1" />
  <xacro:property name="height" value="0.2" />
  <xacro:property name="length" value="0.4" />
  <xacro:property name="wheel_radius" value="0.1" />
  <xacro:property name="wheel_length" value="0.06" />
  <xacro:property name="caster_radius" value="0.05" />
  <xacro:property name="body_inertia">
    <inertia ixx="0.007" ixy="0.0" ixz="0.0" iyy="0.007" iyz="0.0" izz="0.012" />
  </xacro:property>
</robot>
```
### Defining Links and Joints

A link is the part of gazebo that has mass, and can collide with other objects. It's the fundamental building block of gazebo. In a single robot, you may have any number of links, but make sure that there are joints between them. There is no such thing as a "fixed" joint between links, but a workaround is to create a "revolute" link with zero range. In this robot, we make the body and the caster a single link, because then gazebo only calculates dynamics and contacts for the two parts as one, rather than trying to treat them as individual objects. Instead of the caster rolling though, we'll just give it zero friction, and that will simulate the real thing well enough.

Every link must have an inertial block. Otherwise gazebo won't load it. Sometimes, I'll make links with mass of 0.001 kg to get around this, but if it's not defined then the link won't exist in the model. I don't know any other way around this.

```xml
<?xml version="1.0"?>

<robot name="pioneer" xmlns:xacro="https://wiki.ros.org/xacro">
  <xacro:property name="mass" value="0.5755" />
  <xacro:property name="width" value="0.1" />
  <xacro:property name="height" value="0.2" />
  <xacro:property name="length" value="0.4" />
  <xacro:property name="wheel_radius" value="0.1" />
  <xacro:property name="wheel_length" value="0.06" />
  <xacro:property name="caster_radius" value="0.05" />
  <xacro:property name="body_inertia">
    <inertia ixx="0.007" ixy="0.0" ixz="0.0" iyy="0.007" iyz="0.0" izz="0.012" />
  </xacro:property>

  <link name="chassis">
    <inertial>
      <mass value="${mass}" />
      <origin xyz="0 0 0.1"/>
      <xacro:insert_block name="body_inertia"/>
    </inertial>

    <visual name='visual'>
      <origin xyz="0 0 0.1" rpy="0 0 0" />
      <geometry>
        <box size="${body_width} ${body_width} ${body_height}"/>
      </geometry>
    </visual>
    <visual name='visual_caster'>
      <origin xyz="-0.15 0 0.05" rpy="0 0 0" />
      <geometry>
        <sphere radius="${caster_radius}"/>
      </geometry>
    </visual>

    <collision name='collision'>
      <origin xyz="0 0 0.1" rpy="0 0 0" />
      <geometry>
        <box size="${body_width} ${body_width} ${body_height}" />
      </geometry>
    </collision>
    <collision name='collision_caster'>
      <origin xyz="0 0 0.1" rpy="0 0 0" />
      <contact_coefficients mu="0"/>
      <geometry>
        <sphere radius="${caster_radius}"/>
      </geometry>
    </collision>
  </link>

</robot>
```

Notice that in this case the visual and collision blocks are identical. This is not always the case. Often, meshes are used for the visual block, while a primitive is used for the collision. This is helpful because gazebo would have a hard time calculating contacts on a mesh, but the simple primitive is much easier and faster.

### Adding Joints

Now that we have the link, we can add links for both of the wheels, and a joint between them. I'm taking the liberty of limiting both the amount of effort allowed on the joint and limiting the velocity to show, but you can also limit the travel in the same way. More information about urdf joints can be found in the [URDF ROS tutorials](https://wiki.ros.org/urdf/Tutorials).

```xml
<?xml version="1.0"?>

<robot name="pioneer" xmlns:xacro="https://wiki.ros.org/xacro">
  <xacro:property name="mass" value="0.5755" />
  <xacro:property name="width" value="0.1" />
  <xacro:property name="height" value="0.2" />
  <xacro:property name="length" value="0.4" />
  <xacro:property name="wheel_radius" value="0.1" />
  <xacro:property name="wheel_length" value="0.06" />
  <xacro:property name="caster_radius" value="0.05" />
  <xacro:property name="body_inertia">
    <inertia ixx="0.007" ixy="0.0" ixz="0.0" iyy="0.007" iyz="0.0" izz="0.012" />
  </xacro:property>

  <link name="chassis">
    <inertial>
      <mass value="${mass}" />
      <origin xyz="0 0 0.1"/>
      <xacro:insert_block name="body_inertia"/>
    </inertial>

    <visual name='visual'>
      <origin xyz="0 0 0.1" rpy="0 0 0" />
      <geometry>
        <box size="${body_width} ${body_width} ${body_height}"/>
      </geometry>
    </visual>
    <visual name='visual_caster'>
      <origin xyz="-0.15 0 0.05" rpy="0 0 0" />
      <geometry>
        <sphere radius="${caster_radius}"/>
      </geometry>
    </visual>

    <collision name='collision'>
      <origin xyz="0 0 0.1" rpy="0 0 0" />
      <geometry>
        <box size="${body_width} ${body_width} ${body_height}" />
      </geometry>
    </collision>
    <collision name='collision_caster'>
      <origin xyz="0 0 0.1" rpy="0 0 0" />
      <contact_coefficients mu="0"/>
      <geometry>
        <sphere radius="${caster_radius}"/>
      </geometry>
    </collision>
  </link>

  <link name="right_wheel">
    <inertial>
      <mass value="0.1" />
      <origin xyz="0.1 -0.13 0.1" rpy="0 1.5707 1.5707"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.2"/>
    </inertial>

    <visual name="visual">
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
    </visual>

    <collision name="collision">
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
    </collision>
  </link>

  <link name="left_wheel">
    <inertial>
      <mass value="0.1" />
      <origin xyz="0.1 0.13 0.1" rpy="0 1.5707 1.5707"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.2"/>
    </inertial>

    <visual name="visual">
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
    </visual>

    <collision name="collision">
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_wheel_joint" type="revolute">
    <axis xyz="0 1 0"/>
    <limit effort="1000.0" velocity="0.5"/>
    <origin rpy="0 0 0" xyz="0 0.03 0"/>
    <parent link="chassis"/>
    <child link="left_wheel"/>
  </joint>

  <joint name="right_wheel_joint" type="revolute">
    <axis xyz="0 1 0"/>
    <limit effort="1000.0" velocity="0.5"/>
    <origin rpy="0 0 0" xyz="0 -0.03 0"/>
    <parent link="chassis"/>
    <child link="right_wheel"/>
  </joint>

</robot>
```

## Loading the Robot in Gazebo
Because we are using URDF files, rather than SDFs, we load the models a bit differently than as described in the Gazebo tutorials. The easiest way to load a model into gazebo is to use a modified version of the launch file provided in the rotorS_gazebo package developed by the ASL lab at ETH Zurich. Here is a copy of that file (it's found in rotor_gazebo->launch)

```xml
<!-- spawn_mav.launch -->
<launch>
  <arg name="mav_name" default="shredder"/>
  <arg name="model" default="$(find fcu_sim)/urdf/$(arg mav_name)/$(arg mav_name).xacro"/>
  <arg name="tf_prefix" default="$(optenv ROS_NAMESPACE)"/>
  <arg name="x" default="0.0"/>
  <arg name="y" default="0.0"/>
  <arg name="z" default="1.0"/>
  <arg name="enable_ground_truth" default="true"/>
  <arg name="enable_wind" default="true"/>

  <!-- send the robot XML to param server -->
  <param name="robot_description" command="
    $(find xacro)/xacro.py '$(arg model)'
    enable_ground_truth:=$(arg enable_ground_truth)
    mav_name:=$(arg mav_name)"
  />
  <param name="tf_prefix" type="string" value="$(arg tf_prefix)" />

  <!-- push robot_description to factory and spawn robot in gazebo -->
  <node name="spawn_$(arg mav_name)" pkg="gazebo_ros" type="spawn_model"
   args="-param robot_description
         -urdf
         -x $(arg x)
         -y $(arg y)
         -z $(arg z)
         -model $(arg mav_name)"
   respawn="false" output="screen">
  </node>
</launch>
```

If you put your newly created .xacro file in the rotor_gazebo/xacro folder (as described in line 6), then call the spawn_mav launch file with "pioneer" as the "mav_name" argument, it will launch. Here is the example launch file code we use for loading shredder into gazebo

```xml
<launch>

  <arg name="mav_name" default="shredder"/>
  <arg name="world_file" default="cylinders4.world"/>


  <!-- Simulator -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find fcu_sim)/worlds/$(arg world_file)"/>
    <arg name="paused" value="true"/>
    <arg name="gui" value="true"/>
  </include>
  <include file="$(find rotor_gazebo)/launch/spawn_mav.launch">
    <arg name="mav_name"            value="$(arg mav_name)" />
    <arg name="model"               value="$(find fcu_sim)/urdf/$(arg mav_name)/$(arg mav_name)_base.xacro" />
    <arg name="enable_ground_truth" value="true" />
    <arg name="z"                   value="0.08"/>
  </include>
</launch>
```

This is only a snippet from the actual launch file (relative_nav->sim.launch), but if you've done this part right, you should see your robot in the gazebo world.
