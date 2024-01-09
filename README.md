## Drone_Control_and_Navigation

# MATLAB README

## Steps to set up PFC and ePFC Simulink model:

1. Download the developed MATLAB files (`setupPFC.m`, `setupEPFC.m`) and the Simulink models (`ARDronePFC.slx`, `ARDroneEPFC.slx`).

2. Install AR Drone Simulink Development Kit V1.1 Add-On in MATLAB.

3. Go inside the library folder. Add the four developed files in the `/simulation` folder.

4. Double click on `setupPFC.m` or `setupEPFC.m` to view the script in a window. Click on the Run button. Wait for a few seconds as the respective Simulink model will load.

5. A tab with XY graph would load first. Click on the Run button, and the XY trajectory of the drone would be plotted.

# ROS README

**ROS version:** ROS Noetic
**OS:** Ubuntu 20.04

**Developed package:** `epfc_controller`
**Dependency package:** `sjtu_drone`

## Setup dependency package:

```bash
$ sudo apt-get install libignition-math4-dev
$ cd <catkin_ws>/src
$ git clone https://github.com/tahsinkose/sjtu-drone.git
$ cd <catkin_ws>
$ catkin_make

