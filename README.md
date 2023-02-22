# WiFi based landmark etimation
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
