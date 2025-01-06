# Gazebo

## Gazebo Overview

Gazebo is a simulator engine used with ROS.  It uses Ogre3d to render the simulation, and has a built-in physics engine to calculate friction, dynamics, forces, gravity, etc... and can be a very powerful tool for simulating any variety of sensors such as cameras, laser scanners, IMUs, GPS, etc...  We can use the simulator to test ROS code before putting it in hardware, effectively increasing the rate at which we develop code, and help ensure we don't do something stupid with the hardware.

Gazebo loads worlds from xml files called SDF's.  These define all the parts of a robot, the "links" which are the parts of a robot that has mass, the "joints" which hold all the links together, and the "plugins" which are executables attached to links.  These plugins are used for simulating sensors (and for simulating aerodynamics in our case).

The MAGICC lab has simulators built up for multirotor and fixed wing UAVs wrapped around ROSPlane and ROSCopter. These can be found on the MAGICC [github](https://github.com/BYU-MAGICC/). 

