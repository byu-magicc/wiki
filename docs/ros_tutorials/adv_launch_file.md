# Additional Launch File Syntax

## Topic Remapping in Launch Files

When we first learned about launch files we had to do some topic remapping in order to get mimic to work properly. The code is repeated here:

 ``` xml
 <launch>

   <group ns="turtlesim1">
     <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
   </group>

   <group ns="turtlesim2">
     <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
   </group>

   <node pkg="turtlesim" name="mimic" type="mimic">
     <remap from="input" to="turtlesim1/turtle1"/>
     <remap from="output" to="turtlesim2/turtle1"/>
   </node>

 </launch>
```

The syntax above can be used whenever topic remapping is required within launch files. Notice that the syntax above places the remapping inside the node `mimic`.


The syntax below is also used often but don't be confused because in this case the remapping is done outside of the node argument. When using this syntax, the remapping must be called before the node it acts on, or else it won't take effect. This is true for remapping topics and node names.

``` xml
<remap from="/george/turtle1/cmd_vel" to="/turtle1/cmd_vel"/>

<group ns="george">
<node pkg="turtlesim" name="sim" type="turtlesim_node"/>
</group>

```


## Loading Parameters in Launch Files

In the MAGICC Lab we generally use the following format for setting parameters within launch files:
``` xml
<rosparam subst_value="True">
    fabmap:         { visualise: false,
                      SelfMatchWindow: 5,
                      vocab:     $(find openfabmap)/codebooks/R01/vocab.yml,
                      clTree:    $(find openfabmap)/codebooks/R01/clTree.yml,
                      trainbows: $(find openfabmap)/codebooks/R01/trainbows.yml }
    geometry_check: { camera_info_rgb_path:   $(find relative_nav_launch)/param/xtion_rgb.yaml ,
                      camera_info_depth_path: $(find relative_nav_launch)/param/xtion_depth.yaml }
    rgbd_cache:     { camera_info_rgb_path:   $(find relative_nav_launch)/param/xtion_rgb.yaml,
                      camera_info_depth_path: $(find relative_nav_launch)/param/xtion_rgb.yaml }
    hex_map:        { visualization: true,
                      robust_gps: true,
                      robust_lc: true,
                      optimization_rate: 3}
    hlpfollower:    { close_enough_threshold: 1.1}
    hokuyo:         { frame_id: base_laser_link }
    joy_velocity:   { max_velocity: 1.0,
                      max_yaw_rate: 1.5 }
    obstacle_avoid: { max_velocity_command: 1.5,
                      map_downsample: 3,
                      hot_dog_radius: 0.3,
                      bun_radius: 0.5,
                      lookahead_time: 2.0,
                      integral_gain: 0.25,
                      dot_product_gain: 1.0,
                      scale_gain: 1.0,
                      spring_order: 1.0,
                      uncertainty_gain: 1.0 }
    gps_relay:      { drop: 30 }
    rotor_estimator: { use_vo_keyframes: true }
    scan_matcher:   { use_imu: false,
                      use_odom: false }
 </rosparam>
```

In a launch file the above code format results in the following parameters once 'rosparam list' is run.
```
 /fabmap/SelfMatchWindow
 /fabmap/clTree
 /fabmap/trainbows
 /fabmap/visualise
 /fabmap/vocab
 /geometry_check/camera_info_depth_path
 /geometry_check/camera_info_rgb_path
 /gps_relay/drop
 /hex_map/optimization_rate
 /hex_map/robust_gps
 /hex_map/robust_lc
 /hex_map/visualization
 /hlpfollower/close_enough_threshold
 /hokuyo/frame_id
 /joy_velocity/max_velocity
 /joy_velocity/max_yaw_rate
 /obstacle_avoid/bun_radius
 /obstacle_avoid/dot_product_gain
 /obstacle_avoid/hot_dog_radius
 /obstacle_avoid/integral_gain
 /obstacle_avoid/lookahead_time
 /obstacle_avoid/map_downsample
 /obstacle_avoid/max_velocity_command
 /obstacle_avoid/scale_gain
 /obstacle_avoid/spring_order
 /obstacle_avoid/uncertainty_gain
 /rgbd_cache/camera_info_depth_path
 /rgbd_cache/camera_info_rgb_path
 /rosdistro
 /roslaunch/uris/host_hulk__54772
 /rosversion
 /rotor_estimator/use_vo_keyframes
 /run_id
 /scan_matcher/use_imu
 /scan_matcher/use_odom
```

As you can see the parameters are neatly organized into namespaces corresponding to the node which they modify.


### YAML Files

Previously using 'rosparam dump' we created a YAML file which stored parameter values for us.  We can also create this file manually.  We just need to create a file with the .yaml extension which holds all our parameter information.  An example is the following which is contained within ps2.yaml:


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
<rosparam command="load" file="$(find rosparam)/example.yaml"/>
```

For additional information about the launch file format, consult the documentation: https://wiki.ros.org/roslaunch/XML
