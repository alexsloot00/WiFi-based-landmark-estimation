# WiFi based landmark estimation
A quick primer on WiFI based landmark estimation shows two possible approaches exist: using range or bearing information. For both approaches this repository contains options in simulation (MatLab and Gazebo), real-world control based on a Nexus 4WD Mecanum wheel robot.  

## Content
[The WSR toolbox](https://github.com/Harvard-REACT/WSR-Toolbox) created by Hardvard-REACT Lab contains an estimation technique based on bearing informatin using only one WiFi antenna per device.
This repository adds to this work in the form of a Matlab version. This is a simulated environment capable of generating and finding the position of a transmitter using the WSR toolbox principles.
The novel [range-only estimation]() technique is under development by the DTPA Lab in Groningen. A simulation framework is provided in Matlab. Furthermore, a simulated environment created using python with ROS can be used to test this approach in the real-world using the Nexus 4WD Mecanum wheel robot. This repository also allows testing the range-only estimation approach in simulation. Here Gazebo is used in combination with Python and ROS.
Lastly, figures are provided.
The corresponding background information and results can be found in [this paper]().

### Requirements
A linux based system is needed. All files in the repository were written and tested using Ubuntu 20.04.5 LTS. This affects
- the Matlab files for the WSR approach, because of the _.mexa64_ file
- the visualization of the C++ WSR toolbox
- the real-world control and simulations using python for the range-only approach

### Python
To control either the real-world or run a simulation, the following script and possible inputs can be used. If no inputs are provided the default value will be used.
```
FLAGS:
  simulation: bool | default = True         | run a simulation or control a real-world Nexus 4WD
  name: str        | default = nexus_car    | the name of the robot as seen in Gazebo
  port: str        | default = /dev/ttyUSB0 | USB port connected to the real-world Nexus 4WD
```
Option 1: directly invoking python from the scripts directory
```
cd ~/catkin_ws/src/nexus_controller/
python3 main.py -simulation True -name nexus_car -port /dev/ttyUSB0
```
Option 2: using rosrun
```
rosrun main.py -simulation True -name nexus_car -port /dev/ttyUSB0
```

### Matlab
The files concerning the range-only approach can include several different trajectories, e.g. straight, curved and movement orthogonal to the estimated landmark position. To run the WSR toolbox, the main.m and main_csv.m can be run. Where main.m uses self-chosen movement over a given time and main_csv.m uses a predefined movement trajectory and can use real-world collectde CSI data as well. Otherwise, the WiFi signal will be simulated according to the [WSR toolbox paper](https://journals.sagepub.com/doi/full/10.1177/02783649221097989).

### Nexus 4WD robot
As mentioned, the robot used is the Nexus 4 wheel drive Mecanum wheel robot. This robot can move omnidirectional and consists of a [base](https://www.nexusrobot.com/product/4wd-mecanum-wheel-mobile-arduino-robotics-car-10011.html) with 12V DC motors powered by an Arduino. Mounted on the base of the robot is a UP squared board powered by a [XP power board](https://nl.mouser.com/ProductDetail/XP-Power/JCL3012S05?qs=w%2Fv1CP2dgqp6vrT05q%2FO7Q%3D%3D), which in turn is connected to a 12V battery. For simulation, the same robot is used. The model files for the robot as well as the movement, based on forces acting on the base of the robot, were modified from [this repository](https://github.com/RBinsonB/nexus_4wd_mecanum_simulator).
