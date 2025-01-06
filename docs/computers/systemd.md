systemd
=======

Often it is desirable to have your embedded Linux computer run a script or application on startup. Examples include capturing camera video data on a quad when you turn power on to the onboard computer, setting up and running your ROS nodes on a fixed-wing, or setting up the ROS Whirlybird network on a Raspberry Pi.

There are a few ways of running startup scripts which makes the waters more muddy. This is mainly because of historical reasons between **SysV Init**, **Upstart**, and **systemd**. We will use systemd to run userland services.


## Historical Background ##

**UNIX System V** ("System Five") was an early commercial OS developed at AT&T in 1983. They introduces a new style of system startup configuration that we now call *SysV-style init*. Nowadays, Linux still has support for SysV init for backward compatibility even though it was technically replaced in Ubuntu 6.10 by Upstart. Scripts for SysVinit are found in `/etc/init.d` and are run using those executable scripts directly (e.g., `sudo /etc/init.d/apache2 start`).

**Upstart** was the successor to SysVinit and was the default system configuration from Ubuntu 6.10 to 14.10. Upstart was made just to replace SysVinit -- i.e., it did nothing else other than manage processes that you wanted to be able to start and stop at boot or during the system uptime. Upstart introduced non-executable `*.conf` files found in `/etc/init`. These configuration files describe services that can be called with the `start`, `stop`, and `status` commands (e.g., `sudo start apache2`).

While Upstart was the default, a new command showed up on the block that aimed at providing inter-operability to Linux system administrators. This command is the `service` command. It's goal was to provide a common interface to both Upstart and SysVinit configurations and services. It runs as `sudo service apache2 start` and tells you whether it is using the `/etc/init.d/` script or the `/etc/init` Upstart conf file.

With the release of Ubuntu 15.04, Ubuntu switched from upstart to **systemd** as the default configuration manager. systemd is a powerful tool that has caused a lot of [debate](https://www.zdnet.com/article/after-linux-civil-war-ubuntu-to-adopt-systemd/) in the Linux community. Not only does it deal with the starting and stopping of system services, but it takes care of mounting, networking, process management, login, etc. In short, it breaks the UNIX idea of *doing one thing and doing it well*. But, it does run services cleanly and is rather flexible. Systemd service files (called *units*) are found in multiple places, most importantly in `/etc/systemd/system` and are managed with the `systemctl` command (e.g., `sudo systemctl start apache2`). Also, [here](https://unix.stackexchange.com/questions/5877/what-are-the-pros-cons-of-upstart-and-systemd) is a discussion on systemd vs upstart.

## Why not use `/etc/rc.local` ? ##

This script is available and will run Bash scripts during boot, but processes are owned by root and it's really part of the old SysVinit system and isn't that clean.

## Why not use cron? ##

I don't know. It will work and it can be owned by the appropriate user and set with `@reboot`, but it runs in a `/bin/sh` (shell) environment so you have to remember to call `/bin/bash` to get into the right environment. It just doesn't feel right.

## systemd `systemctl` Commands ##

```bash
$ sudo systemctl start myrobot   # the .service extension is optional
$ sudo systemctl stop myrobot
$ sudo systemctl enable myrobot  # installs the service to be run at boot -- requires an [Install] stanza
$ sudo systemctl disable myrobot
```

Read more about managing your system with systemctl [here](https://www.digitalocean.com/community/tutorials/how-to-use-systemctl-to-manage-systemd-services-and-units).

## Example `systemctl` Unit File ##

Here is the service unit file for the ROS Whirlybird setup used in ECEn 483

```bash
# /etc/systemd/system/whirlybird.service
[Unit]
Description="Whirlybird ROS Serial Bridge"
After=network-online.target

[Service]
Type=simple
User=louie
Group=louie
ExecStart=/home/louie/ros_start.sh

[Install]
WantedBy=default.target
```

And here is the `ros_start.sh` script that is run on startup as the `louie` user

```bash
#!/usr/bin/env bash
# Setup the ROS environment for this Whirlybird and start the serial node
#
# This file should be in ~/ros_start.sh and be marked as executable (chmod +x ros_start.sh)

export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=`hostname -I`

source ~/catkin_ws/devel/setup.bash
roslaunch whirlybird_serial serial.launch
```

Before rebooting, you can test the service with

```bash
$ sudo systemctl start whirlybird
```

And you can see if it is running or failed with

```bash
$ sudo systemctl status whirlybird
```

or you can check the log output with

```bash
$ sudo journalctl -u whirlybird
```

Make sure to "install" the service with

```bash
$ sudo systemctl enable whirlybird
```

Reboot and it should start automagically!



More information about writing unit files can be found:

- at [DigitalOcean](https://www.digitalocean.com/community/tutorials/understanding-systemd-units-and-unit-files)
- [these](https://access.redhat.com/documentation/en-US/Red_Hat_Enterprise_Linux/7/html/System_Administrators_Guide/sect-Managing_Services_with_systemd-Unit_Files.html) RedHat docs
- at [this](https://askubuntu.com/questions/676007/how-do-i-make-my-systemd-service-run-via-specific-user-and-start-on-boot) SO question about running as a specific user
- [this](https://answers.ros.org/question/245089/systemd-roslaunch/) ROS-specific SO question about systemd
- Make systemd wait until the network is *really* up using [network-online.target](https://www.freedesktop.org/wiki/Software/systemd/NetworkTarget/)


## Gotchas ##

#### Sourcing `~/.bashrc` Doesn't Work ####

In your equivalent `ros_start.sh` entry point, you may notice that if you try to source your `~/.bashrc` it does not seem to work (i.e., your ROS network setup, aliases, etc). This is because it is being sourced from a non-interactive session and at the top of the `~/.bashrc` you will see code that makes the script bail when that's the case. A solution is to use a secondary script like `~/.bash_profile` to put all of your specific changes in and source that script in `~/.bashrc` for normal use and in the `ros_setup.sh` for robot use. In my opinion, this is the proper way to do that anyways because then you can source control or share the `~/.bash_profile` with other computers/robots.

For the **alias problem** it's because expanding aliases is not the default option in non-interactive mode. This can be changed by adding that option before you source a script:

```bash
$ shopt -s expand_aliases
$ source ~/.bash_profile
```

#### systemd Version 230 Breaks screen and tmux ####

Ubuntu 16.04 ships with systemd version 229 (`systemd --version`). Apparently in version 230, screen and tmux are broken by default which seems silly. Likely, Ubuntu 18.04 will be confusing for users (and us) if systemd keeps this as the default.

Read [here](https://unix.stackexchange.com/questions/5877/what-are-the-pros-cons-of-upstart-and-systemd) for a long discussion of systemd and links about this issue and [here](https://askubuntu.com/questions/802189/how-to-run-tmux-screen-with-systemd-230) for how to change this default behavior.
