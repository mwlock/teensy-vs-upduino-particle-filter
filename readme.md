<a name="readme-top"></a>
<!--
* This readme has been created taking inspiration from the template provided by matthew-william-lock
* https://github.com/matthew-william-lock/teensy-vs-upduino-particle-filter
-->



<!-- PROJECT SHIELDS -->
<!--
*** Using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]
[![LinkedIn][linkedin-shield-matt]][linkedin-url-matthew]
[![LinkedIn][linkedin-shield-zach]][linkedin-url-zach]

<h1 align="center">Evaluating Particle Efficiency of Hardware Accelerated Particle Filtering for Robot Localisation</h1>

[<img src="/images/header.png" width="100%">](https://www.overleaf.com/read/rgdvrxpxgdvm
)

**Keywords**
> Particle filter, Localisation, Hardware Acceleration, Teensy 4.1, UPduino v3.1

<!-- ABOUT THE PROJECT -->
## About The Project

Localisation is a fundamental prerequisite to any meaningful application within the context of robotics. One powerful method for localisation, known as particle filtering, is becoming more and more popular. While particle filtering remains less computationally complex than alternative methods such as Kalman filtering, the ability of a particle filter to accurately and quickly estimate robot pose is directly proportional to the number of particles. This is particularly concerning given that there is an increasing need for particle filters to run in demanding and constrained environments. Consequently, particle filtering methods are required to be fast and efficient under various conditions to allow for scalable implementations in hardware-constrained environments. Since FPGAs and microcontrollers are widely used in various domains, characterisation of the accelerated and non-accelerated per-particle efficiency and performance, in terms of power usage, memory usage, and execution time per sample could serve as a useful guide for future applications. Through this study, it is shown that hardware acceleration should be considered the preferred solution when there is a need for high performance and energy efficiency. It is however noted that the viability of hardware acceleration is dependent on the required map size and resolution. Finally, it is concluded that no value is offered by hardware acceleration for unconstrained applications where high sampling rates are not required.

<!-- GETTING STARTED -->
## Getting Started

Detailed here is the equipment you need to run the provided code and example experiments that we ran for the project.

### Bill of Materials

The two components used for the study are the Teensy 4.1 and the UPduino v3.1. The Teensy 4.1 is a microcontroller that is based on the ARM Cortex M7 processor. The UPduino v3.1 is an FPGA development board that is based on the Lattice iCE40 UltraPlus FPGA. The following table provides a summary of the specifications of the two components.

| Component | Teensy 4.1 | UPduino v3.1 |
| :--- | :---: | :---: |
| Processor | ARM Cortex M7 | Lattice iCE40 UltraPlus FPGA |
| Clock Speed | 600 MHz | 48 MHz |
| Memory | 1 MB | 256 KB |
| Flash Memory | 2 MB | 256 KB |
| GPIO Pins | 34 | 64 |
| Power | 5 V | 3.3 V |

The image below shows the two components used in the study.

<img src="/images/components.png" width="100%">

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


<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->

[contributors-shield]: https://img.shields.io/github/contributors/matthew-william-lock/teensy-vs-upduino-particle-filter.svg?style=for-the-badge
[contributors-url]: https://github.com/matthew-william-lock/teensy-vs-upduino-particle-filter/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/matthew-william-lock/teensy-vs-upduino-particle-filter.svg?style=for-the-badge
[forks-url]: https://github.com/matthew-william-lock/teensy-vs-upduino-particle-filter/network/members
[stars-shield]: https://img.shields.io/github/stars/matthew-william-lock/teensy-vs-upduino-particle-filter.svg?style=for-the-badge
[stars-url]: https://github.com/matthew-william-lock/teensy-vs-upduino-particle-filter/stargazers
[issues-shield]: https://img.shields.io/github/issues/matthew-william-lock/teensy-vs-upduino-particle-filter.svg?style=for-the-badge
[issues-url]: https://github.com/matthew-william-lock/teensy-vs-upduino-particle-filter/issues
[license-shield]: https://img.shields.io/github/license/matthew-william-lock/teensy-vs-upduino-particle-filter.svg?style=for-the-badge
[license-url]: https://github.com/matthew-william-lock/teensy-vs-upduino-particle-filter/blob/master/LICENSE.txt
[linkedin-shield-matt]: https://img.shields.io/badge/-Matthew-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-shield-zach]: https://img.shields.io/badge/-Zacharie-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/matthew-william-lock
[linkedin-url-matthew]: https://linkedin.com/in/matthewwilliamlock
[linkedin-url-zach]: https://www.linkedin.com/in/zacharie-m-97695890/
[product-screenshot]: images/screenshot.png
[Next.js]: https://img.shields.io/badge/next.js-000000?style=for-the-badge&logo=nextdotjs&logoColor=white
[Next-url]: https://nextjs.org/
[React.js]: https://img.shields.io/badge/React-20232A?style=for-the-badge&logo=react&logoColor=61DAFB
[React-url]: https://reactjs.org/
[Vue.js]: https://img.shields.io/badge/Vue.js-35495E?style=for-the-badge&logo=vuedotjs&logoColor=4FC08D
[Vue-url]: https://vuejs.org/
[Angular.io]: https://img.shields.io/badge/Angular-DD0031?style=for-the-badge&logo=angular&logoColor=white
[Angular-url]: https://angular.io/
[Svelte.dev]: https://img.shields.io/badge/Svelte-4A4A55?style=for-the-badge&logo=svelte&logoColor=FF3E00
[Svelte-url]: https://svelte.dev/
[Laravel.com]: https://img.shields.io/badge/Laravel-FF2D20?style=for-the-badge&logo=laravel&logoColor=white
[Laravel-url]: https://laravel.com
[Bootstrap.com]: https://img.shields.io/badge/Bootstrap-563D7C?style=for-the-badge&logo=bootstrap&logoColor=white
[Bootstrap-url]: https://getbootstrap.com
[JQuery.com]: https://img.shields.io/badge/jQuery-0769AD?style=for-the-badge&logo=jquery&logoColor=white
[JQuery-url]: https://jquery.com 

[report]: https://www.overleaf.com/read/rgdvrxpxgdvm
