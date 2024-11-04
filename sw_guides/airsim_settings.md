# Airsim Settings
Airsim settings are stored in a json file. The settings control many aspects of the simulation:
- Number and type of vehicles
- Sensors on each vehicle
- Simulation speed
- etc.

The most common settings are shown here. For full documentation on AirSim settings, visit the [AirSim documentation](https://microsoft.github.io/AirSim/settings/).

## Launching airsim with a settings file

To launch AirSim with a settings json file, specify it with the `--settings` flag. For example:

```
 ./CityBlocks.sh --settings ~/airsim_settings.json -windowed
 ```

(Note that some AirSim flags use single dashes, such as `-windowed`, and some use double dashes, such as `--settings`.)

If the provided settings file is invalid or missing, it AirSim will launch with default settings and ask whether you want a car simulation or multirotor simulation.

## Common Settings

These are settings that you are likely to use at the MAGICC lab. For full documentation on AirSim settings, visit the [AirSim documentation](https://microsoft.github.io/AirSim/settings/). There is a separate page for [non-camera sensors](https://microsoft.github.io/AirSim/sensors/). This is a wiki, please expand this page as you find information.

## Simulator Settings

### SettingsVersion

All settings file must have a "SettingsVersion" field. For now, use `1.2`.

### ClockSpeed

This allows you to slow down or speed up the simulation. This is particularly helpful if your computer is struggling with a complex simulation. This can increase the effective framerates of simulated cameras.

### ViewMode

This sets how the main viewport camera moves around. There are a number of options listed in the AirSim documentation. `SpringArmChase` is a good default; it will follow the vehicle around. By setting ViewMode to "NoDisplay", the main display of AirSim is made blank. This is useful for saving computing resources once you have the simulation set up the way you need it.

### Wind

Sets the wind for the simulation.

### Recording

AirSim has some built-in recording tools, which record vehicle pose and images. They do not record sensors without modifying the source code and rebuilding, and so recording from ROS may be more practical.

## Vehicle Settings

AirSim supports any number of vehicles, and they can be a mix of types. When there is only one vehicle it is used by default, and when there are multiple vehicles methods referencing them must specify the vehicle.

### VehicleType

There are three options for multirotors: `SimpleFlight`, `PX4Multirotor`, and `ArduCopter`. `SimpleFlight` uses the built-in controller to fly the vehicle and has a number of different control options. The others work with their respective software.

This can also be set to `ComputerVision`, which disables all vehicle mechanics. The location of the "vehicle" can be set through the software API.

This can also be set to `PhysXCar` or `Ardurover`. These are cars, which are like drones that don't fly.

### Cameras

See below.

### X, Y, Z, Roll, Pitch, Yaw

This sets the initial location and attitude of the vehicle. The origin is also set to this location. The coordinate frame is note rotated by roll, pitch, or yaw.

## Camera Settings

Note that in AirSim cameras are configured separately from other sensors. Each vehicle can have as many cameras as you want. Many of the important settings are under `CaptureSettings`.

### CaptureSettings.ImageType

Airsim supports several types of cameras. Normal cameras are `0` or scene cameras. Other options include depth and IR cameras and segmentation view.

### CaptureSettings.Width, CaptureSettings.Height

Size in pixels.

### CaptureSettings.FOV_Degrees

This sets the horizontal field of view of the camera. The conversion between FOV and focal length is

$$
FOV = 2tan^{-1}(\frac{w}{2f})
$$

. Note that you can't convert between horizontal and vertical FOV by dividing by the image aspect ratio (For example, to simulate a rotated camera). Instead convert to focal length and back.

### Gimbal

The gimbal settings enable image stabilization in any or all axes.

### X, Y, Z, Roll, Pitch, Yaw

These set the pose of the camera w.r.t. the vehicle. By default, the camera is at the center of the vehicle, facing forward. Through the magic of computer graphics, the vehicle is transparent from the inside, but parts of the vehicle (quadrotor landing gear) may be visible if you don't move the camera.

## Sensor Settings

Multirotors come with an IMU by default. Adding an IMU manually allows you to specify noise settings. Available sensors are: barometer, IMU, GPS, magnetometer, distance sensor, and lidar.

### IMU

Note that AirSim uses continuous noise and drift parameters for the IMU, rather than discrete noise parameters, and so you will have to convert.

