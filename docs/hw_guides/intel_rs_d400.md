***
## Intel RealSense D400 Camera
***
D415 and D435
### Firmware Update
***
1. Boot into Windows (sorry)
2. Download the firmware update tool from the Intel website (https://downloadcenter.intel.com/download/27514/Windows-Device-Firmware-Update-Tool-for-Intel-RealSense-D400-Product-Family)
3. Follow instructions on the site to install latest firmware

### Force Hardware Reset on Startup
***
As of June 13 2018, the RealSense driver has a bug that after launching the realsense node once, the camera crashes
and needs to be unplugged and plugged back in to work. This is a work around to force the camera to reset
on startup so that it works.

Replace `realsense2_camera_manager/src/realsense_node_factory.cpp` with [this file](assets/realsense_node_factory.cpp)
and replace `realsense2_camera_manager/include/realsense_node_factory.h` with [this file](assets/realsense_node_factory.h)

Then rebuild your catkin workspace


### Restart Depth Auto exposure
***
As of 6-1-18 the RealSense driver has a bug that it thinks the IR auto-exposure is turned on when it is not.
This can be fixed by toggling the setting using
```
rosrun rqt_reconfigure rqt_reconfigure
```
but that can be annoying everytime you start the camera up, so this is a work around to force it to toggle the
rs435_depth_enable_auto_exposure.

first you will need to install the timed_roslaunch package from source and put it in your workspace

```
git clone https://github.com/MoriKen254/timed_roslaunch.git
```

Then put the following lines in your launch file

```
<include file="$(find timed_roslaunch)/launch/timed_roslaunch.launch">
  <arg name="time" value="4" />
  <arg name="pkg" value="mapping_3d" />
  <arg name="file" value="ir_exp_off.launch" />
  <arg name="node_name" value="depth_auto_exp_off" />
</include>

<include file="$(find timed_roslaunch)/launch/timed_roslaunch.launch">
  <arg name="time" value="5" />
  <arg name="pkg" value="mapping_3d" />
  <arg name="file" value="ir_exp_on.launch" />
  <arg name="node_name" value="depth_auto_exp_on" />
</include>
```
but change the value in `<arg name="pkg" value="mapping_3d" />` to whatever package you put the following launch files in

Make two new launch files in your workspace called `ir_expo_off.launch`
```
<launch>
<!--turn off Depth auto exposure-->
  <node pkg="dynamic_reconfigure" type="dynparam" name="set_ir_autoexposure_off"
        args="set camera/realsense2_camera_manager rs435_depth_enable_auto_exposure 0">
  </node>
</launch>
```
and `ir_expo_on.launch`
```
<launch>
  <!-- Turn on depth auto exposure -->
  <node pkg="dynamic_reconfigure" type="dynparam" name="set_ir_autoexposure_on"
        args="set camera/realsense2_camera_manager rs435_depth_enable_auto_exposure 1">
  </node>
</launch>
```
NOTE:to work on a RealSense D415 camera, change `rs435_depth_enable_auto_exposure` in the above files to `rs415_depth_enable_auto_exposure`

it is kind of a lame workaround, but at least it works.

### Tips and tricks for better quality
The D435's optimal depth resolution is 848x480 and the D415's optimal depth resolution is 1280x720. The depth images will be the most accurate and least noisy at these resolutions.
[This link](https://realsense.intel.com/wp-content/uploads/sites/63/BKMs-For-Tuning-RealSense_D4xx_Cameras_WP_1.7.pdf) has some other tips for tuning for better performance
