# Motion Capture Tutorial

## Prerequisite

1. Setting markers for the object you want to track

2. Connect to the Magicc network

    Turn Between the lab conference room and the flight room (146A and 146C), there's a Netgear box used for the mocap system. To turn it on, press the power button marked "**MoCap**" that is under the desk. *Pressing the wrong button might cause serious problems!* And don't forget to turn it off when you're finished.

3. Open [Motive](https://optitrack.com/software/motive/) (can be found on the Windows desktop). ***Make sure Motive is not blocked by the firewall!***


## Tracking the vehicle/object

1. In the menu, select "View -> Assets Pane".

2. On the left sidebar check the boxes, and you should see the markers on the perspective view. Choose the markers you want to track. 

    In this case, "*DemonstrationCopter*" is being tracked (containing four markers).
    ![Mocap motive 1](figures/mocap_motive1.png)



## Data Collecting with ROS

1. Install the [vrpn_mocap package](https://index.ros.org/r/vrpn_mocap/) following the instructions in the link.

2. Source your `ros2` installation.
```bash
source /opt/ros/<ros-distro>/setup.bash
```

3. Run the `vrpn_mocap` node.
```bash
 ros2 launch vrpn_mocap client.launch.yaml server:=<IP-of-mocap-machine> port:=3883 
```

    ```bash
    INFO] [launch]: All log files can be found below /home/jacob/.ros/log/2025-07-25-11-07-10-205177-ros-box.jacob-1292937
    [INFO] [launch]: Default logging verbosity is set to INFO
    [INFO] [client_node-1]: process started with pid [1292938]
    [client_node-1] [INFO] [1753463231.307355248] [vrpn_mocap.vrpn_mocap_client_node]: Created new tracker ROScopter
    [client_node-1] [INFO] [1753463231.310954372] [vrpn_mocap.vrpn_mocap_client_node]: Creating sensor 0
    ```

3. The tracked object's pose information is publishing to your device through ROS topics.
```bash
 ros2 topic list
```
```
/vrpn_mocap/DemonstrationCopter/pose
/parameter_events
/rosout
```

!!! tip
    Use `ros2 topic hz /<topic_name>` in a new terminal to see the rate that the messages are coming in.

```bash
 ros2 topic echo /vrpn_mocap/DemonstrationCopter/pose
```
```
header:
  stamp:
    sec: 1753467069
    nanosec: 39448569
  frame_id: world
pose:
  position:
    x: 0.02851906791329384
    y: 1.4423145055770874
    z: -0.5593897700309753
  orientation:
    x: -0.0014903525589033961
    y: -0.10285155475139618
    z: -0.002313516102731228
    w: -0.9946929216384888
---
```

## Data Collecting with the Motive Record Function

1. Make sure you're under the **LIVE** mode. Click on the red recording button. Click on it again to end the recording.
    ![Mocap motive 2](figures/mocap_motive2.png)

2. Click on "EDIT" to switch to the **EDIT** mode. Select "File -> Export Tracking Data", and save the file in the CSV format.
    ![Mocap export tracking data](figures/mocap_export_data.png)

    Your file should look like this:
    ![Mocap csv file](figures/mocap_csvfile.png)

### When you are finished, be sure to unplug the mocap system box from power.
