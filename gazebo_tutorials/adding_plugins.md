# Adding Plugins

## Adding Plugins

Plugins are compiled executables which are installed in gazebo. You can attach these plugins to links on your robot and they can pretty much do anything you can imagine. They are able to access pretty much any information about the simulation, and can publish ROS messages, apply forces, move joints, etc... Typically, we have used them to simulate sensors and simulate aerodynamics, but they could certainly be used for other things.

In this tutorial, we will have you add an RGBD camera to your robot.

We have gathered a number of useful xacro files that make this a lot easier. Look in the fcu_sim meta package, in the fcu_sim_plugins package. In the xacro file, there will be a number of macros which allow you to quickly attach a plugin to your robot.

```
rotor_simulator
+-- attitude_controller
+-- joy_sim
+-- rotor_gazebo
+-- rotor_gazebo_plugins
|    |-- CMakeLists.txt
|    |-- package.xml
|    +-- include
|    +-- src
|    |-- xacro
|    |    |-- rgbd.xacro
|    |    |-- wind.xacro
|    |    |-- imu.xacro
|    |    |-- ...
|    |    |
+-- sim_reset
```

## rgbd.xacro
Let's look at the rgbd.xacro file. It is split into two macros, first , on line 6 defines the "xtion" macro

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="https://wiki.ros.org/xacro">
  <xacro:property name="pi" value="3.14159265359" />

  <!-- XTION -->
  <xacro:macro
    name="xtion"
    params="namespace
            parent_link">
    <xacro:depth_camera
      namespace="${namespace}"
      parent_link="${parent_link}"
      camera_suffix=""
      frame_rate="30"
      depth_range="6.5"
      rgb_range=""
      image_topic="/camera/rgb/image_raw"
      depth_topic="/camera/depth/image_raw"
      image_camera_info_topic="/camera/rgb/camera_info"
      depth_camera_info_topic="/camera/depth/camera_info"
      pointcloud_topic="/camera/pointcloud">
      <origin xyz="0.127 0 -0.0889" rpy="0 0 0" /> <!-- for shredder -->
    </xacro:depth_camera>
  </xacro:macro>
  ...
```
This sets up the parameters for the plugin to match the characteristics of the xtion camera, it then calls the generic rgbd camera macro defined on line 26, there is a macro which defines a generic RGBD camera.
```xml
  <!-- Macro to add a depth camera. -->
    <xacro:macro
    name="depth_camera"
    params="namespace
            parent_link
            camera_suffix
            frame_rate
            depth_range
            rgb_range
            image_topic
            depth_topic
            image_camera_info_topic
            depth_camera_info_topic
            pointcloud_topic
            *origin">
  ...
```
Because there are so many parameters to change in the generic macro, it is easier to build a wrapper around that macro which sets the parameters for a known sensor. All you need to do is to include the xacro file in your new URDF, and call the xtion macro, setting the namespace and parent link appropriately.

## Adding the Plugin
Let's go ahead and add the rgbd camera to the pioneer robot.

```xml
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

  <link name="right_wheel"/>
    <inertial>
      <mass value="0.1" />
      <origin xyz="0.1 -0.13 0.1" rpy="0 1.5707 1.5707"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.2"/>
    </inertial>

    <visual name="visual"/>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
    </visual>

    <collision name="collision"/>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
    </collision>
  </link>

  <link name="right_wheel"/>
    <inertial>
      <mass value="0.1" />
      <origin xyz="0.1 0.13 0.1" rpy="0 1.5707 1.5707"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.2"/>
    </inertial>

    <visual name="visual"/>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
    </visual>

    <collision name="collision"/>
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
    <child link="left_wheel"/>
  </joint>

  <xacro:include filename="$(find fcu_sim_plugins)/xacro/rgbd.xacro"/>
  <xacro:xtion namespace="" parent_link="chassis"/>

</robot>
```
