# P Controller with Gazebo
~~~~
<launch>

  <arg name="mav_name" default="shredder"/>

 <!-- Set non-default parameters -->
 <rosparam subst_value="True">

 </rosparam>

 <!--Low Level Path Planning -->
 <!-- your node goes here -->

 <!--Control-->
 <rosparam command="load" file="$(find rotor_controller)/gains/pid.yaml"/>
 <node name="pid_controller"      pkg="rotor_controller"    type="rotor_controller_pid"  output="screen"/>


  <!-- Simulator -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find rotor_gazebo)/worlds/basic.world"/>
    <arg name="paused" value="false"/>
    <arg name="gui" value="true"/>
  </include>
  <node name="laser_tf" pkg="tf" type="static_transform_publisher" args="0 0 0 0 0 0  /laser_link /base_laser_link 100"/>
  <node name="laser_tf2" pkg="tf" type="static_transform_publisher" args="0 0 0 0 0 0 /shredder/ground_truth /laser_link 100"/>
  <include file="$(find rotor_gazebo)/launch/spawn_mav.launch">
    <arg name="mav_name"            value="$(arg mav_name)" />
    <arg name="model"               value="$(find rotor_gazebo)/urdf/$(arg mav_name)_base.xacro" />
    <arg name="enable_logging"      value="false" />
    <arg name="enable_ground_truth" value="true" />
    <arg name="log_file"            value=""/>
    <arg name="z"                   value="1.0"/>
  </include>
  <group ns="$(arg mav_name)">
    <node name="attitude_controller" pkg="attitude_controller" type="attitude_controller_node" output="screen" >
      <rosparam command="load" file="$(find rotor_gazebo)/param/$(arg mav_name).yaml"/>
      <remap from="odometry" to="ground_truth/odometry"/>
      <remap from="command" to="/command"/>
    </node>
    <node name="sim_reset" pkg="sim_reset" type="sim_reset_node">
      <remap from="joy" to="/joy"/>
      <remap from="gazebo/set_model_state" to="/gazebo/set_model_state"/>
    </node>
  </group>

  <!-- Truth -->
  <node name="truth" pkg="rotor_estimator" type="truth">
    <param name="use_vo_keyframes" value="true"/>
    <remap from="pose" to="$(arg mav_name)/ground_truth/pose_with_covariance"/>
  </node>

</launch>
~~~~
