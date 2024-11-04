# VTOL AirSim Quickstart Guide

This guide is for running a simulation of a tiltrotor VTOL vehicle inside a prebuilt binary of the CityBlocks world - a custom world modified for AirSim by the MAGICC Lab. The driving script, complete with trajectory generation, controllers, and an AirSim client, is [`geometric_control_airsim_sim.py`](https://gitlab.magiccvs.byu.edu/urbanmobility/vtolsim/vtolsim/-/blob/master/geometric_control/geometric_control_airsim_sim.py), located in the [vtolsim](https://gitlab.magiccvs.byu.edu/urbanmobility/vtolsim/vtolsim) repository.

Here is what you will need to do to run the script `geometric_control_airsim_sim.py`:

1. Download the latest CityBlocks zip archive [here](https://byu.box.com/s/4so60ynxgdxmvwrmiytldkifr3favqqh) and unzip it to wherever you like
1. Create a file at `~/Documents/AirSim/settings.json` - or modify it if it already exists - with the following settings (see the example `settings.json` below):
    * `"SimMode": "Vtol"`
    * a vehicle with `"VehicleType": "vtolsimple"`
1. Clone the BYU-MAGICC fork of Airsim somewhere - `git@github.com:byu-magicc/AirSim` or `https://github.com/byu-magicc/AirSim`
1. If you haven't already, create a python virtual environment specifically for AirSim (it will make everything simpler for AirSim)
    * e.g. `python -m venv ~/.virtualenvs/airsim`
1. Activate your AirSim virtual environment
    * e.g. `source ~/.virtualenvs/airsim/bin/activate`
1. `cd` into `AirSim/PythonClient` and run `pip install -e .`
1. Now clone vtolsim: `git clone --recursive git@magiccvs.byu.edu:urbanmobility/vtolsim/vtolsim.git`
1. `cd` into `vtolsim/geometric_controller`
1. In another terminal session, start running `<path to CityBlocks dir>/LinuxNoEditor/CityBlocks.sh`
1. Now you can run `python geometric_control_airsim_sim.py` and it should fly the trajectory.

### Example `settings.json`:
```json
{
  "SettingsVersion": 1.2,
  "SimMode": "Vtol",
  "ClockSpeed": 1.0,
  "LogMessagesVisible": false,
  "Vehicles": {
    "uav0": {
      "VehicleType": "vtolsimple"
    }
  }
}
```

Notes:

* Lower the `ClockSpeed` setting if your machine is struggling to run vtolsim + AirSim.
    * e.g. `"ClockSpeed": 0.6` to run the simulation at 60% real time
* Set `LogMessagesVisible: true` if you need that, but having it off is better for recording video.
