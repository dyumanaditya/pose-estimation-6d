# Building the Pose Estimation Node

## Requirements
This package will build **only** with:
- Ubuntu 18.04
- ROS Melodic

## Dependencies
This package depends on:
- [Azure-Kinect-Sensor-SDK](https://github.com/microsoft/Azure-Kinect-Sensor-SDK)
- [Azure-Kinect-ROS-driver](https://github.com/microsoft/Azure_Kinect_ROS_Driver)
- [PCL (Point Cloud Library)](https://pointclouds.org/)

### Setup Azure-Kinect-Sensor-SDK
Copy and paste each of the lines below one at a time into a terminal:
```bash
curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -

sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod

sudo apt-get update

sudo apt-get install k4a-tools

sudo apt-get install libk4a1.4-dev

cd /etc/udev/rules.d/

sudo nano 99-k4a.rules
```
Copy the following block and place it inside the file `99-k4a.rules`. This is to connect with the Azure Kinect Camera without root permissions.

```
# Bus 002 Device 116: ID 045e:097a Microsoft Corp.  - Generic Superspeed USB Hub
# Bus 001 Device 015: ID 045e:097b Microsoft Corp.  - Generic USB Hub
# Bus 002 Device 118: ID 045e:097c Microsoft Corp.  - Azure Kinect Depth Camera
# Bus 002 Device 117: ID 045e:097d Microsoft Corp.  - Azure Kinect 4K Camera
# Bus 001 Device 016: ID 045e:097e Microsoft Corp.  - Azure Kinect Microphone Array

BUS!="usb", ACTION!="add", SUBSYSTEM!=="usb_device", GOTO="k4a_logic_rules_end"

ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097a", MODE="0666", GROUP="plugdev"
ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097b", MODE="0666", GROUP="plugdev"
ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097c", MODE="0666", GROUP="plugdev"
ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097d", MODE="0666", GROUP="plugdev"
ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097e", MODE="0666", GROUP="plugdev"

LABEL="k4a_logic_rules_end"
```
Save the file and exit `nano` editor. The Azure-Kinect-Sensor-SDK should now be setup.


### Setup Azure-Kinect-ROS-driver
You will need to clone this repository at the same level as you clone the Pose Estimation package.
To create a catkin workspace paste these lines one at a time in a terminal:
```bash
mkdir catkin_ws

cd catkin_ws

mkdir src

catkin_make

cd src

git clone https://github.com/microsoft/Azure_Kinect_ROS_Driver

```
You will need to use the same catkin workspace later on.


### Setup PCL
Enter the following command into the terminal to install PCL:
```bash
sudo apt install libpcl-dev
```

## Building the ROS package
To build the Pose Estimation package, copy the following lines one by one into a terminal. Make sure you are inside the same `catkin_ws/src` you initialized while setting up the Azure-Kinect-ROS-driver:

```bash
cd catkin_ws/src

git clone -b catkin_pkg https://gitlab.com/arjun.98/pose_estimation_6d

cd ..

catkin_make
```
The ROS package should now be built. Please refer to [usage guide](../docs/USAGE.md) for more information on how to use the node.