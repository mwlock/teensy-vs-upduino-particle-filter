# Evaluating the Particle Efficiency Between the Teensy 4.1 and UPduino v3.1 FPGA for Robot Localisation

*Documentation is not finalised*

- [Installation](#installation)
  * [Requirements](#requirements)
  * [Getting Started](#getting-started)
- [Teensy Particle Filter](#teensy-particle-filter)
- [Teensty Particle Filter + FPGA Accelerator](#teensty-particle-filter---fpga-accelerator)
- [Dualsense Controller](#dualsense-controller)
- [Webots Simulator](#webots-simulator)
- [For Interest's Sakes](#for-interest-s-sakes)
  * [Teensy Development](#teensy-development)
- [Interesting Tools](#interesting-tools)

<small><i><a href='http://ecotrust-canada.github.io/markdown-toc/'>Table of contents generated with markdown-toc</a></i></small>


<small><i><a href='http://ecotrust-canada.github.io/markdown-toc/'>Table of contents generated with markdown-toc</a></i></small>

## Installation

### Requirements

- [Ubuntu 22.04]()
- [Numpy]()
- [Webots 2022a](https://github.com/cyberbotics/webots/releases/tag/R2022a)

### Getting Started

```bash
some text
```

## Teensy Particle Filter

Steps to run the Teensy particle filter & simulation:

1. Connect your Teensy 4.1 to your computer via USB!

![Teensy 4.1](https://www.pjrc.com/store/teensy41_4.jpg)

2. Build, upload and run the code on the Teensy 4.1 as shown in the [Teensy 4.1 Readme](https://github.com/matthew-william-lock/teensy-vs-upduino-particle-filter/tree/fpga-accelerator/src/teensy_particle_filter).

3. Install docker 

```bash
sudo snap install docker
```

> This step is shown for Ubuntu, please see the [docker documentation](https://docs.docker.com/engine/install/) for other operating systems.


4. Run the micro-ROS agent on your host machine

```bash
sudo docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO serial --dev /dev/ttyACM0 -v6
```

*Documentation incomplete*

## Teensty Particle Filter + FPGA Accelerator

## Dualsense Controller

Should you wish controll the e-puck with a playstation 5 dualsense controller, follow the following steps.

Firstly, we need to follow the installation procedure for ```pydualsense``` as shown [here](https://github.com/flok/pydualsense).

1. Install pydualsense

```
sudo apt install libhidapi-dev
```

```
pip install --upgrade pydualsense
```

2. Give your user the appropriate permissions

```
sudo chmod -R 777 /dev
```

3. Clone and install the dualsense driver.

Make sure to carry out these steps in your ```ros2_ws```

```
git clone https://github.com/matthew-william-lock/ros2-ds
colcon build
source install/local_setup.bash
```

Launch teleop for the controller 
```
ros2 launch p9n_node teleop.launch.py
```

4. Run your experiment, but be sure to disable the random bound by setting ```rb:=false```. See the exampe below

```
ros2 launch mcl mcl_teensy_launch.py rviz:=true mission_time:=10 rb:=false
```

## Webots Simulator

## For Interest's Sakes

### Teensy Development
- [VSCode + PlatformIO](https://platformio.org/)
- [micro_ros_arduino_pub_sub_example](https://github.com/botamochi6277/micro_ros_arduino_pub_sub_example)

## Interesting Tools
- [The MRPT project](https://github.com/MRPT/mrpt)
- [OctoMap](https://github.com/OctoMap/octomap)
