Installing ROS Kinetic on a Raspberry Pi Zero W
===============================================

The Raspberry Pi Zero W is a lightweight, inexpensive computer capable of running Linux. It also has bluetooth and WiFi, making it a good option for using on small UAV platforms, especially when you need to communicate with the ROSFlight board remotely for tuning gains, saving data, etc. However, it is only able to run Raspbian distros, meaning it can be kind of a pain to get ROS to run on it. This guide will explain how to install ROS Kinetic on the Raspberry Pi Zero W. To my knowledge, it isn't possible to install Melodic on the Zero, and if it is possible, there isn't much documentation for it, so we're stuck with Kinetic for now.

## Setting Up The OS ##

The easiest way to install the OS is to flash your SD card with a predownloaded image. Raspbian Lite images can be found [here](https://downloads.raspberrypi.org/raspbian_lite/images/). It is (highly) recommended that you install a version of Jessie instead of Buster. Buster will have issues with finding boost and you will have to install it manually, which is a huge pain.
Download the `.zip` file. Before you can flash the image, you need to know the name of the SD card partition. Without the SD card inserted, run `lsblk`. Then insert the card and run `lsblk` again to see the name of the SD card partition (the one that wasn't there before). It should be something like `/dev/mmcblk0` or `/dev/sdX`. Then, to flash the image, run the command

```bash
unzip -p /PATH/TO/IMAGE.zip | sudo dd of=/dev/sdX bs=4M conv=fsync
```

replacing `/PATH/TO/IMAGE.zip` with the path to the downloaded zip file and  `/dev/sdX` with the name of SD card partition. After a few minutes, the OS will be ready to run.

If anything breaks, more detailed instructions are found on the [Raspberry Pi website](https://www.raspberrypi.org/documentation/installation/installing-images/linux.md).

## Setting Up SSH ##

Unless you feel like finding a monitor, USB hub, micro HDMI cable, mouse, and keyboard, you'll need to set up SSH to be able to do anything with the Zero. This is super easy to do. You just need to add an empty text file called `ssh` to the root directory of the partition. This can be done with `touch ssh` when in the proper directory. Additionally, to get the Zero to connect to Wifi, create a text file called `wpa_supplicant.conf` in the same directory and paste these contents into it:

```bash
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
country=US

network={
    ssid="Your SSID"
    psk="YourWPAPassword"
    key_mgmt=WPA-PSK
}
```
inserting the proper SSID and password of the network you want to connect to.

Now, eject the SD card, insert it into the Zero's SD slot, and boot the Raspberry Pi. It should connect to the internet automatically (assuming the network is available). To ssh into it, from your own machine connected to the same network run

```bash
$ ping raspberrypi.local
```

to find the IP address of the Pi, then run

```bash
$ ssh pi@<IP_ADDRESS>
```

to connect. The default password is `raspberry`.

## Installing ROS Kinetic ##

Now for the fun part. The ROS wiki has a [really good tutorial](https://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi) demonstrating how to install ROS Kinetic on a Raspberry Pi running Raspbian. You can follow these instructions with a few caveats.

#### 1. ####

For ROS to build successfully, you need to have the `yaml-cpp` package installed. This is easily accomplished with

```bash
$ sudo apt install libyaml-cpp-dev
```

#### 2. ####

Make sure you run the commands associated with the correct version of Raspbian (if you heeded my previous warning, that would be Jessie).

#### 3. ####

In Section 2.1, the key they have shown is out of date. So instead of running

```bash
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
```

run

```bash
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

#### 4. ####

You will only need the ROS-Comm package (not the desktop package) on the Pi. However, there are a whole bunch of packages that ROSFlight requires that you'll want to install with ROS-Comm. These packages include `std_msgs`, `std_srvs`, `geometry_msgs`, `eigen_stl_containers`, `sensor_msgs`, `tf`, and `nav_msgs`. Additionally, compilation of the package `collada_urdf` will fail. You could use the workaround they mention, but it takes forever and `collada_urdf` isn't necessary for ROSFlight, so you can just ignore it. So rather than running the command

```bash
$ rosinstall_generator ros_comm --rosdistro kinetic --deps --wet-only --tar > kinetic-ros_comm-wet.rosinstall
$ wstool init src kinetic-ros_comm-wet.rosinstall
```

you'll run something like

```bash
$ rosinstall_generator ros_comm std_msgs std_srvs geometry_msgs eigen_stl_containers sensor_msgs tf nav_msgs --rosdistro kinetic --deps --wet-only --exclude collada_parser collada_urdf --tar > kinetic-ros_comm-wet.rosinstall
$ wstool init src kinetic-ros_comm-wet.rosinstall
```

including whatever other ROS packages you think you'll need.

If you find out you need other packages in the future, you can follow the instructions in Section 4.2 to install new packages. Note that you'll have to rebuild ROS each time, so figure out all the packages you want and install them at the same time so you don't have to keep rebuilding.

#### 5. ####

When you build the workspace in Section 3.3, you're going to need to add swap space. Open `/etc/dphys_swapfile` and change the line `CONF_SWAPSIZE=100` to `CONF_SWAPSIZE=1024`. I think this is generally not recommended unless you're using a separate drive, but it didn't break when I did it, so you can find and mount a separate drive or you can be lazy like me and it'll probably be fine.


Assuming that built correctly, you should be good to go! You can now build your catkin workspaces. If you find out you need additional ROS packages, just follow the steps in Section 4.2 on the ROSberry Pi tutorial page.
