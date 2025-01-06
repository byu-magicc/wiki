# Quickstart Airism Guide 
### (Using Only Precompiled Binaries) 

This guide details the quickest way to get up and running using Airsim and its python API on Linux.  Please see the [official Airsim documentation](https://microsoft.github.io/AirSim/) for more depth on how to utilize Airsim.

There are four installation steps:
1. Download and extract a precomplied environment available on the [Airsim Github repo](https://github.com/microsoft/AirSim/releases).  My favorite is the [Neighborhood Environment](https://github.com/microsoft/AirSim/releases/download/v1.4.0-linux/AirSimNH.zip).
2. Download and extract the latest Airsim source code available on the [Airsim Github repo](https://github.com/microsoft/AirSim/releases).  Right now, this version [1.4.0](https://github.com/microsoft/AirSim/archive/v1.4.0-linux.zip).
3. Navigate to the Airsim source code directory and run `./setup.sh` followed by `./build.sh`.
4. (Note: Do not do this last step if you use Jupyter Notebooks or iPython.  We are working on a fix for this, but for now there is a breaking dependency conflict with these packages.  This can be avoided by using vitual python environments, which is more complex than this guide goes.)  Run `pip install Airsim`.

You are now ready to run an Airsim simulation!  Below is an example python script for using Airsim for waypoint navigation.

```python
import airsim

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

while True:
    in_coords = list(map(int,input("\nEnter destination waypoint in NED cooridnates ").strip().split()))[:3]
    print("Flying to ({},{},{}) at 5 m/s".format(in_coords[0],in_coords[1],in_coords[2]))
    # client.moveToPositionAsync(in_coords[0], in_coords[1], in_coords[2], 5) # This will set the multirotor on a path and imediately return
    client.moveToPositionAsync(in_coords[0], in_coords[1], in_coords[2], 5).join() # This will block until the multirotor has reached its destination
    in_char = input("Continue flying?  y/n ")
    if in_char == 'y':
        continue
    elif in_char == 'n':
        break

client.armDisarm(False)
client.reset()
client.enableApiControl(False)
```

In one terminal window run the executable buried deep in the precompiled environment's file structure.  For example, the neighborhood executable can be found at `<path_to_your_download_location>/Neighborhood/AirSimNH/Binaries/Linux/AirSimNH`.  Select multirotor when prompted.  In a seperate terminal window run the provided python script to control the simulated multirotor.

That's it!  Refer to the [official documentation](https://microsoft.github.io/AirSim/) for more in depth guides on utiziling all of Airsim's capabilities.

### Extra Examples

As an aside, here are two more example python scripts for working with Airsim.  The first enables camera output to be fed into opencv (be sure to change the camera name from `0` to the camera name you have listed in the Airsim settings found in `~/Documents/AirSim/settings.json`).  

```python
import numpy as np
import airsim
import cv2

client = airsim.MultirotorClient()
client.confirmConnection()

cv2.namedWindow('image', cv2.WINDOW_NORMAL)
inkey = 'a'

while inkey != 27:
    in_image = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])[0]
    img1d = np.fromstring(in_image.image_data_uint8, dtype=np.uint8) # get numpy array
    image = img1d.reshape(in_image.height, in_image.width, 3) # reshape array to 3 channel image array H X W X 3
    cv2.imshow('image', image)
    inkey = cv2.waitKey(1)

cv2.destroyAllWindows()
```

The second implements basic keyboard control for the multirotor using the `keyboard` package (`pip install keyboard`, be sure to run this script as `sudo`)

```python
import numpy as np
import airsim
import keyboard

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

client.takeoffAsync().join()

print('Multirotor Initialized')
print('Fly with the following controls')
print('w: forward')
print('s: backward')
print('a: left')
print('d: right')
print('left arrow: yaw left')
print('right arrow: yaw right')
print('up arraw: up')
print('down arrow: down')
print('hold left shift to go faster')
print('press esc to cancel flight and land')

running = True
control_freq = 60 # control update frequency in hertz
control_period = 1/control_freq
base_speed = 15 
base_rotation_rate = 75 
velocity = np.array([0.0, 0.0, 0.0])
heading = 0.0
yaw_rate = 0.0

while running:
    state = client.getMultirotorState()
    velocity[0] = 0.0
    velocity[1] = 0.0
    velocity[2] = 0.0
    yaw_rate = 0.0
    q = state.kinematics_estimated.orientation.to_numpy_array()
    heading = np.arctan2(2.0*(q[2]*q[3] + q[0]*q[1]), 
                         q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3])

    if keyboard.is_pressed('esc'):
        running = False
        continue
    else:
        if keyboard.is_pressed('left_arrow'):
            yaw_rate = -base_rotation_rate
            client.rotateByYawRateAsync(yaw_rate, control_period)
            continue
        elif keyboard.is_pressed('right_arrow'):
            yaw_rate = base_rotation_rate 
            client.rotateByYawRateAsync(yaw_rate, control_period)
            continue
        if keyboard.is_pressed('w'):
            velocity[0] += np.cos(heading)
            velocity[1] += np.sin(heading)
        if keyboard.is_pressed('s'):
            velocity[0] -= np.cos(heading)
            velocity[1] -= np.sin(heading)
        if keyboard.is_pressed('a'):
            velocity[0] += np.sin(heading)
            velocity[1] -= np.cos(heading)
        if keyboard.is_pressed('d'):
            velocity[0] -= np.sin(heading)
            velocity[1] += np.cos(heading)
        if keyboard.is_pressed('up_arrow'):
            velocity[2] -= 1
        if keyboard.is_pressed('down_arrow'):
            velocity[2] += 1

        norm = np.sqrt(np.sum(velocity**2))
        if norm != 0.0:
            velocity /= norm 
        velocity *= base_speed
        if keyboard.is_pressed('left_shift'):
            velocity *= 2
        velocity[0] = (velocity[0]+state.kinematics_estimated.linear_velocity.x_val)/2
        velocity[1] = (velocity[1]+state.kinematics_estimated.linear_velocity.y_val)/2

        client.moveByVelocityAsync(velocity[0], velocity[1], velocity[2], control_period)

client.armDisarm(False)
client.reset()
client.enableApiControl(False)
```

