# WiFi based landmark estimation
A quick primer on WiFI based landmark estimation shows two possible approaches exist: using range or bearing information. For both approaches this repository contains one option in simulation (MatLab and Gazebo), and real-world control based on a Nexus 4WD Mecanum wheel robot.  

## Content
The novel [range-only estimation](https://arxiv.org/abs/2304.08182) technique is under development by the DTPA Lab in Groningen. A simulation framework is provided in Matlab. This repository also allows testing the range-only estimation approach in simulation. Here Gazebo is used in combination with Python and ROS.

[The WSR toolbox](https://github.com/Harvard-REACT/WSR-Toolbox) created by Hardvard-REACT Lab contains an estimation technique based on bearing information using only one WiFi antenna per device. This repository adds to this work in the form of a Matlab version. This is a simulated environment capable of generating and finding the position of a transmitter using the WSR toolbox principles.
Furthermore, a simulated environment created using python with ROS can be used to test both approaches in the real-world using the Nexus 4WD Mecanum wheel robot. It was found CSI can not be used to obtain the distance between the receiver and transmitter reliably, therefore, the range-only landmark localization does not yet work operationally in the real world, it currently uses the true andmark position which should be unknown.

Lastly, the CSI data and figures obtained from simulations and real-world experiments are provided.
The corresponding background information and results can be found in [this paper]().

### Requirements
A linux based system is needed. All files in the repository were written and tested using Ubuntu 20.04.5 LTS. This affects
- the Matlab files for the WSR approach, because of the extension to code written in C
- the visualization of the C++ WSR toolbox
- the real-world robot control due to the use of ROS 
- the real-world collection of CSI data using the Linux kernel

### Matlab
The files concerning the range-only approach include several different trajectories, e.g. straight, curved and movement orthogonal to the estimated landmark position. The type of movement is specified at the top of the file. The gain (>0) can be chosen by the user, however, if the gain does not satisfy the constraints the estimated landmark will not converge to the true position. 
To simulate the WSR toolbox method, run the file `main.m`. Here, a predefined movement trajectory can be used or a trajectory can be created in the form of a list of [x; y; z] coordinates. Additionally, a JSON file containing CSI data can be used if the data corresponding to the movement is also obtained from the same experiment. If no CSI data is provided, the code will choose to use emulated CSI data using the formula from the [WSR toolbox paper](https://journals.sagepub.com/doi/full/10.1177/02783649221097989). If CSI data is chosen the code will show compariosns between the given CSI data and the emulated data. Next to the CSI data and trajectory, the user can choose to creat a simple (2D computed) profile or create a 3D profile by setting the variable `simple` to 1 or 0, respectively. If no CSI data is given, using the 3D profile is recommended for better accuracy, the elevation angle can simply be ignored. Lastly, the resolution of the profile (default 360x180) can be set by changing the resolution of the gammaList and betaList. However, this method is computationally expensive, so beware.

### Python
# Installation
This repository containts all the python files from [this repository](https://github.com/alexsloot00/nexus_controller), which has been used to directly commit to GitHub. Installation of this separate repository using the README.md ensures a correct installation of the python, ROS and catkin workspace.

# Running the code
To control either the real-world or run a Gazebo simulation, the following script and associated inputs (flags) can be used. If no inputs are provided the default value will be used. For movement, options include forward, backward, right, left, and circle. None of the inputs need to be invoked with "", the code will handle this. Additionally, when using the distance-only estimator, the chosen trajectory will be dynamically optimized based on the current position, instead of using the inputted movement.
```
FLAGS:
  simulation: bool | default = True           | simulate or control a real-world Nexus 4WD
  estimator: str   | default = WSR            | choose distance-only (DO) or WSR estimator
  name: str        | default = "nexus_car"    | the name of the robot as seen in Gazebo
  port: str        | default = "/dev/ttyUSB0" | USB port connected to the real-world Nexus 4WD
  velmag: float    | default = 0.1            | magnitude of the velocity in meters/second
  timestep: float  | default = 0.05           | the time step between consecutive loop iterations
  move: str        | default = "circle"       | movement of the robot, distance based on velmag
  runtime: float   | default = 10.0           | the time of runnning the program in seconds
```
Option 1: directly invoking python from the scripts directory
```
cd ~/catkin_ws/src/nexus_controller/
python3 main.py 
python3 main.py -simulation True -name nexus_car -port /dev/ttyUSB0 -velmag 0.1 -timestep 0.05 -move circle -runtime 10.0
```
Option 2: using rosrun
```
rosrun nexus_controller main.py
rosrun nexus_controller main.py -simulation True -name nexus_car -port /dev/ttyUSB0 -velmag 0.1 -timestep 0.05 -move circle -runtime 10.0
```

### WSR 
The [modified WSR C++ toolbox](https://github.com/alexsloot00/WSR-Toolbox-cpp) contains the installation guide and how to use the WSR toolbox.

### Nexus 4WD robot
As mentioned, the robot used is the Nexus 4 wheel drive Mecanum wheel robot. This robot can move omnidirectional and consists of a [base](https://www.nexusrobot.com/product/4wd-mecanum-wheel-mobile-arduino-robotics-car-10011.html) with 12V DC motors controlled by an with the base included [Arduino 328 controller](https://www.google.com/search?q=Arduino+328+Controller&hl=en-US&tbm=shop&source=lnms&sa=X&ved=2ahUKEwiO1LnMlpD-AhU1hv0HHa5jDBoQ_AUoAnoECAoQBA&biw=2560&bih=1272&dpr=1). Mounted on the base of the robot is a [UP squared board](https://nl.mouser.com/ProductDetail/AAEON-UP/UPS-APLP4F-A20-0432?qs=hd1VzrDQEGi10dyhG9RNMA%3D%3D) powered by a [XP power board](https://nl.mouser.com/ProductDetail/XP-Power/JCL3012S05?qs=w%2Fv1CP2dgqp6vrT05q%2FO7Q%3D%3D), which in turn is connected to a [battery](https://www.bol.com/nl/nl/p/hacker-lipo-accupack-11-1-v-5800-mah-aantal-cellen-3/9300000035249404/?Referrer=NLGOOFS&utm_source=google&utm_medium=free_shopping). The UP squared board is connected to an [external SSD hard drive](https://nl.mouser.com/ProductDetail/Kingston/OCP0S31024Q-A0?qs=4ASt3YYao0X4fy01e9yEQw%3D%3D) for booting [Ubuntu (full) Desktop 18.04.0](https://old-releases.ubuntu.com/releases/18.04.0/) and extra storage. An [Intel 5300 NIC](https://nl.aliexpress.com/item/32435835710.html?pdp_npi=2%40dis%21EUR%21%E2%82%AC%2010.29%21%E2%82%AC%2010.29%21%21%21%21%21%40211b813c16806093148537781e1273%2155556538860%21btf&_t=pvid%3A5d6e949f-5f60-42f5-9cdf-871499ea2e44&afTraceInfo=32435835710__pc__pcBridgePPC__xxxxxx__1680609315&spm=a2g0o.ppclist.product.mainProduct&gatewayAdapt=glo2nld) is connected to the UP squared board and screwed on using an [extension from half size to full size](https://www.dustinhome.nl/product/5010788918/mini-pci-express-half-size-full-size-adapter?gclid=EAIaIQobChMI09jg35WQ_gIVEpBoCR0z5g9UEAQYAiABEgJTm_D_BwE&utm_medium=cpc&utm_source=google). Lastly, the [antenna](https://nl.mouser.com/ProductDetail/Linx-Technologies/ANT-W63WS2-SMA?qs=Li%252BoUPsLEnvWNQpgFXXzfQ%3D%3D) is attached to the WiFi card.

For simulation, the same robot is used. The model files for the robot as well as the movement, based on forces acting on the base of the robot, were modified from [this repository](https://github.com/RBinsonB/nexus_4wd_mecanum_simulator).
