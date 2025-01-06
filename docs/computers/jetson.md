NVIDIA Jetson TX2
=================

## Setup ##

When you unbox an NVIDIA Jetson TX2, it will come pre-flashed with Linux4Tegra (27.1, at the time of this writing). However none of the libraries/tools will be installed (i.e., CUDA, cuDNN, etc). Install JetPack 4.2.x on a host machine running (vanilla) Ubuntu 18.04. (You can attempt this process on other distributions or version numbers, but there are no guarantees of a working setup. We've got it running on Ubuntu Budgie 18.04, but it is finicky.)

## Software Overview ##

#### JetPack ####

JetPack is the Jetson SDK that is installed on the Host machine to push libraries and an OS image onto the TX2.

#### Linux4Tegra (L4T) ####

Linux with a kernel that has been patched specifically for Tegra devices.

#### OpenCV4Tegra ####

A Tegra CPU-optimized closed-sourced version of OpenCV. It is not CUDA-enabled (which seems rather silly...).

#### CUDA-Enabled OpenCV ####

See the installation instructions [here](../sw_guides/opencv.md).

#### ROS Melodic ####

We have not had certificate issues on JetPack 4.2.x, so we recommend following the [general install procedure on the ROS website](https://wiki.ros.org/melodic/Installation/Ubuntu).

If you are interested and/or have issues, Jetson Hacks has a nice (albeit outdated) article illustrating how to [install ROS on the TX2](https://www.jetsonhacks.com/2018/04/27/robot-operating-system-ros-on-nvidia-jetson-tx-development-kits/). Their `installROS.sh` script available in their [repo](https://github.com/jetsonhacks/installROSTX2) is essentially the standard ROS installation procedure, just with an extra line that can fix supposedly messed up certificates.

## Host Computer ##

The host computer is the machine on which you install JetPack and which you use to flash the OS image and other libraries to the TX2. Although it is possible to run JetPack on Ubuntu 'Flavors,' e.g. Ubuntu Budgie, there are some weird quirks in the installation procedure. It is much easier if you use 'vanilla' Ubuntu 18.04 with JetPack 4.2.x.


**The following is a legacy issue that has not been encountered on JetPack 4.2.x. We leave the instructions here for edge cases that might crop up:**

You **may** find your host computer's `apt` package manager is doing weird things after you install JetPack and setup a TX2. If you find that `sudo apt update` leaves you with the following errors:

```bash
Reading package lists... Done
N: Skipping acquire of configured file 'non-free/binary-arm64/Packages' as repository 'http://repository.spotify.com stable InRelease' doesn't support architecture 'arm64'
N: Skipping acquire of configured file 'main/binary-arm64/Packages' as repository 'https://desktop-download.mendeley.com/download/apt stable InRelease' doesn't support architecture 'arm64'
E: Failed to fetch http://us.archive.ubuntu.com/ubuntu/dists/xenial/main/binary-arm64/Packages  404  Not Found [IP: 91.189.91.26 80]
E: Failed to fetch http://us.archive.ubuntu.com/ubuntu/dists/xenial-updates/main/binary-arm64/Packages  404  Not Found [IP: 91.189.91.26 80]
E: Failed to fetch http://us.archive.ubuntu.com/ubuntu/dists/xenial-backports/main/binary-arm64/Packages  404  Not Found [IP: 91.189.91.26 80]
E: Failed to fetch http://security.ubuntu.com/ubuntu/dists/xenial-security/main/binary-arm64/Packages  404  Not Found [IP: 91.189.88.152 80]
E: Some index files failed to download. They have been ignored, or old ones used instead.
```

then the following workaround (found [here](https://devtalk.nvidia.com/default/topic/1004601/jetson-tx2/after-installing-jetpack3-0-host-computer-is-not-able-to-apt-get-update/)) can be applied. Essentially, it seems that the installation of JetPack has opened `apt` to looking for some **arm64** packages. However, your host machine is (likely) an **x86_64** (AKA, **amd64**) architecture. To disable `apt` from looking for **arm64** versions of standard Ubuntu packages, edit the `/etc/apt/sources.list` (and possibly others in the `/etc/apt/sources.list.d` directory) to be limited to **amd64** as follows:

```bash
deb [arch=amd64] <url>
```

## Carrier Board: ConnectTech Orbitty ##

The Orbitty is fully compatible with both the TX1 and the TX2. See the compatibility report [here](https://connecttech.com/resource-center/kdb344-cti-nvidia-jetson-carrier-board-tx2-tx1-compatibility/).

To use the Orbitty with the TX2, you will have to install COnnectTech Inc's (CTI) board support package (BSP), which is called [CTI-L4T](https://connecttech.com/resource-center/cti-l4t-nvidia-jetson-board-support-package-release-notes/). *<-- Read the whole thing! It has links to answer questions.* At the time of this writing, CTI-L4T-V126 has been installed on the TX2/Orbitty successfully. Note that upon installing the BSP for your specific carrier board, the TX2 will not boot on other carrier boards, including the dev kit.

To install CTI-L4T, you can follow the [instructions](https://connecttech.com/resource-center/kdb373/), or walk through the steps laid out in the next subsection. You can read through the flow on the instructions web-page, but we have extensively tested the steps in this document, so probably just follow them. The installation steps below have been completed successfully using an Ubuntu 18.04 x86_64 host machine. To then install CUDA and the other packages that JetPack normally installs on the target Jetson TX2, run the Nvidia SDKManager again. On the *STEP 01: Development Environment* screen, uncheck "Host Machine" and change "Target Hardware" to "Jetson TX2 (P3310)," or whatever your applicable hardware is. The next screen is *STEP 02: Details and License*. Under **Target Components**, uncheck **Jetson OS**. This will prevent JetPack from overwriting the BSP changes, while still allowing JetPack to actually push CTI-L4T and any other components (like CUDA) under the **Jetson SDK components** section over to the TX2 device.

Some have asked why we uncheck "Host Machine". It just installs a bunch of software (CUDA, Computer Vision {opencv?}, & Developer Tools) that you probably do not need on your host machine. Their versions of CUDA and OpenCV probably are not the ones you want (see our versions: [CUDA](../sw_guides/install_cuda.md), [OpenCV](../sw_guides/opencv.md)), and I would venture to bet you are not developing for Nvidia. If you are still curious, Google what exact software and their versions are included. IMHO, if I do not need it to successfully install software on the TX2, I do not need their automated versions clogging up my host machine resources with background processes. and stomping on my own carefully installed versions of libraries.

## Instructions: Start to Finish ##

If you have the Jetson TX2 on a Developer Kit, you will need a T-10H Security Torx screw bit to remove the TX2 from the dev kit (or a small flat head, just do not bend the screwdriver).

1. Connect the TX2 to the Orbitty.

1. Install the dependencies for the Nvidia SDK Manager: `sudo apt install libgconf-2-4`

1. Download the [Nvidia SDK Manager](https://developer.nvidia.com/embedded/jetpack). This will allow you to select and download your targeted version of JetPack (e.g. 4.2.2).

1. Download the [Orbitty CTI-L4T BSP](https://connecttech.com/resource-center/l4t-board-support-packages/) (e.g. CTI-L4T-V126.tgz). Make sure that the BSP from CTI plays nicely with your chosen JetPack from NVIDIA. For example, JetPack 4.2.2 ships L4T 32.2.1 for the TX2, so get the CTI-L4T BSP targeting that release of L4T, and make sure it supports the Orbitty. Some CTI-L4T releases only support a small subset of CTI carrier boards. Check compatibility using the release notes and download page links above. More information is available [here](https://connecttech.com/support/resource-center/nvidia-jetson-tx2-tx1-product-support/).

1. Run SDKManager, but only to download the files
   * Under *STEP 01: Development Environment*
    * Uncheck "Host Machine"
    * Select your correct "Target Hardware" (e.g. "Jetson TX2 (P3310)")
    * Select your target JetPack Version (e.g. JetPack 4.2.2)
   * Under *STEP 02: Details and License*
    * Leave "Jetson OS" checked
    * Uncheck "Jetson SDK Components" & "Additional SDKs"
    * Under the "Download & Install Options" drop-down menu, change both "Download folder" and "Target HW image folder" options to easily accessible, and JetPack version unique, locations (e.g. `~/jetson422/sdkm_downloads` and `~/jetson422/nvidia_sdk`)
    * Accept the Terms & Conditions
    * Continue to Step 03
    * After the download completes, the installation screen will appear (where it asks how the TX2 is connected)
    * Cancel the JetPack setup here

1. Make sure the TX2 is plugged into power

1. Place the TX2 in recovery mode
    * Connect a micro usb 2.0 cable between the Orbitty Carrier board and your Host machine
    * Press & hold the "Recovery" button
    * Press & release the "Reset" button
    * Release the "Recovery" button

1. Now that JetPack has pulled the relevant OS files to your local host machine, we need to patch the L4T kernel with driver support for the Orbitty carrier board using the CTI-L4T-V1xx BSP. The instructions are found [here](https://connecttech.com/resource-center/cti-l4t-nvidia-jetson-board-support-package-release-notes/#bkb-h8). You **must** follow the **Manual Flashing** instructions because the automatic script fails to flash the image (most times, but you can try if you like). Now, read the bright-orange "Attention box." These instructions will actually build the patched OS image and flash it to the internal eMMC on the TX2.

    Attention:
    When you are ready to call the `./flash.sh <profile> mmcblk0p1` command as instructed in the **Manual Flashing** commands from the CTI instruction page, as of JetPack 4.2.2, you first need to run `sudo ./flash.sh ./cti/tx2/orbitty mmcblk0p1`. This will likely also fail, stalling with print statements in the terminal similar to:
    ```bash
    ...
    [   4.3455 ] Boot Rom communication
    [   4.3463 ] tegrarcm_v2 --chip 0x18 0 --rcm rcm_list_signed.xml
    [   4.3469 ] BootRom is not running
    [   9.6155 ]
    [  10.6191 ] tegrarcm_v2 --isapplet
    [ 1020.5594 ]
    [ 1020.5768 ] tegradevflash_v2 --iscpubl
    [ 1020.5780 ] CPU Bootloader is not running on device.
    [ 2036.3675 ]
    [ 2037.3711 ] tegrarcm_v2 --isapplet
    [ 3052.1754 ]
    [ 3052.1779 ] tegradevflash_v2 --iscpubl
    [ 3052.1801 ] CPU Bootloader is not running on device.
    [ 4067.9835 ]
    [ 4068.9871 ] tegrarcm_v2 --isapplet
    ...
    ```
    (If it does not fail/stall, you will see something like the following, and you may skip the rest of this block):
    ```bash
    ...
    [   0.3570 ] Copying signatures
    [   0.3576 ] tegrahost_v2 --chip 0x18 0 --partitionlayout flash.xml.bin --updatesig images_list_signed.xml
    [   0.4085 ]
    [   0.4086 ] Boot Rom communication
    [   0.4097 ] tegrarcm_v2 --chip 0x18 0 --rcm rcm_list_signed.xml
    [   0.4104 ] BootRom is not running
    [   5.4682 ]
    [   6.4715 ] tegrarcm_v2 --isapplet
    [   6.4736 ] Applet version 01.00.0000
    [   6.4761 ]
    [   6.4762 ] Sending BCTs
    [   6.4784 ] tegrarcm_v2 --download bct_bootrom br_bct_BR.bct --download bct_mb1 mb1_bct_MB1_sigheader.bct.encrypt
    [   6.4792 ] Applet version 01.00.0000
    [   6.4814 ] Sending bct_bootrom
    [   6.4815 ] [................................................] 100%
    [   6.4830 ] Sending bct_mb1
    [   6.4841 ] [................................................] 100%
    [   6.5002 ]
    [   6.5002 ] Generating blob
    ...
    ```
    Obviously you do not have to wait 4000 seconds to know it has stalled; I ran it this long for illustration. Best practice is to wait at least 90 seconds before determining it has stalled. If it does stall at the `tegrarcm_v2 --isapplet` print statement, check the printout history for something like:
    ```bash
    ...
    4354: RAW:     6275072(   1532 blks) ==>  4231167028:6275084
    4355: SKP:        4096(      1 blks) ==>  4237442112:12
    4356: RAW:       81920(     20 blks) ==>  4237442124:81932
    4357: SKP:  1935671296( 472576 blks) ==>  4237524056:12
    -- Total: ---------------------------------------------------
    4358 CHUNK 30064771072(7340032 blks) ==>  4237524068(1034539 blks)

    done.
    system.img built successfully.
    ...
    ```
    If you see similar lines, you may quit the stalled command with `CTRL + C`. (If you do not see them, I hope you said your morning prayers. Also, go get some help.) Next, run `sudo ./flash.sh -r ./cti/tx2/orbitty mmcblk0p1`. Note the `-r`! It is different than above. This will use the pre-built image from the previous install attempt, instead of building a new one. This allows the `flash.sh` script to start sending the image before the pseudo-network between the host and TX2 times out. If it fails to flash, on the Orbitty, try: (a) press & hold the "Recovery" button, (b) press & release the "Reset" button (c) release the "Recovery" button a maximum of 0.3 seconds before pressing enter to run the flash command in the terminal. Because `sudo` is required for the flash command, first run a generic `sudo update` so the `sudo ./flash.sh ...` command does not pause waiting for your admin password. This also helps the pseudo-network timing.

1. Now that Linux is actually flashed on the TX2, unplug the USB cable, press the "Reset" button on the Orbitty, and the TX2 should boot up into the Linux config screen. Accept the license, select a username, computer hostname, and password, and finish the setup process.

    Note:
    It may appear to hang during this process (specifically at a "waiting for unattended upgrade..." dialog box). Leave it be for at least an hour, if not overnight. It is not actually frozen. If you want a way to avoid waiting, you need to turn off "unattended upgrades" by patching a file in the `rootfs/etc/apt` directory in the Jetson system image source files on the host system before running the flash utility on the command line the final time. If you do not know what that means, just be patient and let it flash normally. You can really screw up your image if you do not know what your doing in this process.

1. Reboot the TX2.

1. Plug the micro USB cable back into the TX2.

1. Start "SDK Manager" again, on the host machine.

1. Make sure not to overwrite the OS image on the *STEP 02* screen by un-checking **Jetson OS**, but leave the **Jetson SDK components** & **Additional SDKs** checked.

1. Press *Continue to STEP 03*, and follow the on screen instructions, entering first the *Host* machine password, then the username and password you configured for the TX2. JetPack should start copying the relevant packages (CUDA, etc) over to the TX2 and installing them. This can take a while. Go get a coffee.

    Note:
    If it fails, shut everything off, disconnect power, and boot everything back up, and try the SDKs again. It has happened once that I needed to re-flash the OS image a second time before the SDKs would successfully install.

1. Make it "Headless" to save resources when flying. Follow the instructions [here](headless.md).

1. You're all done!

![coming together](assets/coming_together.gif)

## Clocks and Such ##

#### Power Profiles: `nvpmodel` ####

The Jetson TX2 has 5 power profiles that can be set with the following. You may also want to include the `jetson_clocks` command:

```bash
$ sudo nvpmodel -m [mode]
$ sudo jetson_clocks
```

To see which mode is currently being used, run

```bash
$ sudo nvpmodel -q
```

You will probably want the highest power mode, using all available cores on the TX2.:

```bash
$ sudo nvpmodel -m 0
$ sudo jetson_clocks
```

Good info on the NVIDIA Power Model (nvpmodel): [JetsonHacks - nvpmodel](https://www.jetsonhacks.com/2017/03/25/nvpmodel-nvidia-jetson-tx2-development-kit/).

**The following information is applicable to JetPack <= 4.2.1, as far as I can tell. Using JetPack 4.2.2, if you set the nvpmodel once, your selected setting is persistent across a power cycle. If you are using Wi-Fi, instead of an Ubiquiti Bullet, you will still likely need to create the systemd service to turn off power saving on the Wi-Fi, but this is untested.**

It is advantageous to run this command on boot. You can do this with a `systemd service` file. First, create a `jetson-max-power.sh` file with execute permissions. (For security, it is recommended to place it in a directory owned by root, such as `/etc/<max_power.d>/jetson-max-power.sh`. Personally, I just place it at `~/software/jetson-max-power.sh`.) Wherever you place the `*.sh` script, make sure the `ExecStart` line in the `*.service` file points to it.

The [`*.service`](https://gitlab.magiccvs.byu.edu/lab/wiki/blob/master/public/computers/assets/jetson-max-power.service) and [`*.sh`](https://gitlab.magiccvs.byu.edu/lab/wiki/blob/master/public/computers/assets/jetson-max-power.sh) files are available on the [MAGICC GitLab server](https://gitlab.magiccvs.byu.edu/lab/wiki/tree/master/public/computers/assets). Place the `*.service` file at `/etc/systemd/system/jetson-max-power.service`, and enable with `sudo systemctl enable jetson-max-power.service`.

NOTE: CRITICAL
    Be sure to fix any relevant paths in these two files before enabling the systemd service. If you have already enabled, and need to change the `*.service` file, you will need to either reboot, or run `sudo systemctl daemon-reload`.

## WiFi and Connectivity ##

The Jetson TX2 comes with embedded Wi-Fi/bluetooth. The dev kit comes with two antennas. The TX2 Wi-Fi can run with just 1 antenna connected to J9, which is the U.FL connector furthest from the S/N sticker. (See [here](https://devtalk.nvidia.com/default/topic/919406/can-the-tx1-be-used-with-only-1-antenna-/).) The other U.FL connector is for Bluetooth.

If you are only going to use a Ubiquiti Bullet hard-wired to the Orbitty ethernet port (this is highly recommended for MAGICC Lab Vehicles! see [Lab Standard HW/SW Setup](https://internal.magiccvs.byu.edu/#!inventory-hw.md)), we suggest disabling the Wi-Fi in the gui, or alternatively hard-blocking the wireless card with `rfkill`.

You can view the Wi-Fi RSSI/link quality with the following command (with or without the `watch -n 0.1`):

```bash
$ watch -n 0.1 iw dev wlan0 link
```

It has also been noted that the RF chip has a power-saving feature which could introduce lag (see [here](https://devtalk.nvidia.com/default/topic/995819/jetson-tx1/jetson-tx1-wifi-speed-and-latency/)). It can be turned off with:

```bash
$ sudo iw dev wlan0 set power_save off #- to disable power save and reduce ping latency.
```

You can add this line to the `.sh` script called by `jetson-max-power.service`. If you downloaded the file available on the GitLab server, it is already included. You're welcome. :)

## USB Devices ##

Note! **Update**
    As of JetPack 4.2.x, the following drivers are all included in the kernel by default. This information is left here for legacy use cases.

**Original Content**

It is likely that your USB devices do not work correctly out of the box with the TX2. This is especially the case for a flip32 board for ROSflight. To fix this you need to build some drivers in with the kernel. This process is pretty straight-forward.

To build the kernel with additional USB drivers, follow the instructions and video found [here](https://www.jetsonhacks.com/2017/07/31/build-kernel-ttyacm-module-nvidia-jetson-tx2/). This video shows the ACM module being added, however there are a few additional drivers you will likely require. These include:
- USB Winchiphead CH341 Single Port Serial Driver
- USB Modem (CDC ACM) support
- USB CP210x family of UART Bridge Controllers

After following the instructions to add these drivers, reboot your TX2 and your USB devices should show up in /dev/ttyUSB? or /dev/ttyACM? as you would expect.

Update: With Jetpack 3.2 and L4T 28.2, ttyACM devices appear to work out of the box, but you may still need to follow the above instructions if your device doesn't work. Also, please note that there is an updated video for [building the kernel for L4T 28.2](https://www.jetsonhacks.com/2018/03/22/build-kernel-modules-nvidia-jetson-tx2-2/).

## Cloning ##

Warning!
    The following commands and information has not been tested on L4T 32.x.x as installed by using the JetPack 4.2.x software. Use at your own peril.

The following commands only clone the OS partition, and do not affect the bootloader, so it is recommended that you first set up your new TX2 with the above instructions if you are going to be using an Orbity or other carrier board. Once both TX2's have the same bootloader, you can follow these steps for cloning derived from [this guide](https://elinux.org/Jetson/TX2_Cloning).

First, make sure you have at least 50GB of free space on the volume where you have Jetpack installed. cd into the directory containing the L4T installation package on the host PC. The command below will save the TX2's eMMC image to the specified file on the host.

```bash
$ sudo ./flash.sh -r -k APP -G backup.img jetson-tx2 mmcblk0p1
```

In this case, we call the file backup.img, so the same flash.sh script can be re-used to format and flash other Jetson's with the image. Copy the .raw file which containts the image of the OS partition to where the flashing script can find it. You may want to backup the vanilla image already in the bootloader directory.

```bash
$ sudo cp backup.img.raw bootloader/system.img
```

The recommended way to restore multiple units with different serial numbers is to save the image above as "system.img" and use the head L4T flashing script, flash.sh, with the -r option (to reuse your backed-up system.img without rebuilding the vanilla image from scratch):

```bash
$ sudo ./flash.sh -r -k APP jetson-tx2 mmcblk0p1
```

## Intel Realsense D400 Series ##

Below are some guides to help you get started using the Intel Realsense D400 series cameras on the TX2. (Some of this may be out-of-date, likely targeting L4T 28.x.x and below):

 - First and foremost, our own wiki page: [Intel RealSense D400 Camera](../hw_guides/intel_rs_d400.md)
 - [Intel RealSense Camera Installation – NVIDIA Jetson TX2](https://www.jetsonhacks.com/2017/03/26/intel-realsense-camera-installation-nvidia-jetson-tx2/)
 - [Github jetsonhacks/buildLibrealsense2TX](https://github.com/jetsonhacks/buildLibrealsense2TX)
 - [Intel RealSense Package for ROS on NVIDIA Jetson TX2](https://www.jetsonhacks.com/2017/03/29/intel-realsense-package-for-ros-on-nvidia-jetson-tx2/)

## Sources and Drivers ##

For JetPack >=4.2.x:
Sources: https://developer.nvidia.com/embedded/dlc/l4t-jetson-driver-package-32-1-JAX-TX2
Drivers: https://developer.nvidia.com/embedded/dlc/l4t-sources-32-1-JAX-TX2

For JetPack <4.2.x:
Sources: https://developer.nvidia.com/embedded/dlc/l4t-jetson-driver-package-28-3-tx2
Drivers: https://developer.nvidia.com/embedded/dlc/l4t-sources-28-3-tx2
