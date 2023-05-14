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
*Click the image below to read the project report*

[<img src="/images/header.png" width="100%">](https://www.overleaf.com/read/rgdvrxpxgdvm
)

**Keywords**
> Particle filter, Localisation, Hardware Acceleration, Teensy 4.1, UPduino v3.1

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#bill-of-materials">Bill of Materials</a></li>
        <li><a href="#prerequisites">Prerequisites</a></li>
      </ul>
    </li>
    <li>
      <a href="#usage">Usage</a>
      <ul>
        <li><a href="#simulated-localisation">Simulated Localisation</a></li>
        <li><a href="#embedded-localisation">Embedded Localisation</a></li>
      </ul>
    </li>

    
  </ol>
</details>

<!-- ABOUT THE PROJECT -->
## About The Project

Localisation is a fundamental prerequisite to any meaningful application within the context of robotics. One powerful method for localisation, known as particle filtering, is becoming more and more popular. While particle filtering remains less computationally complex than alternative methods such as Kalman filtering, the ability of a particle filter to accurately and quickly estimate robot pose is directly proportional to the number of particles. This is particularly concerning given that there is an increasing need for particle filters to run in demanding and constrained environments. Consequently, particle filtering methods are required to be fast and efficient under various conditions to allow for scalable implementations in hardware-constrained environments. Since FPGAs and microcontrollers are widely used in various domains, characterisation of the accelerated and non-accelerated per-particle efficiency and performance, in terms of power usage, memory usage, and execution time per sample could serve as a useful guide for future applications. Through this study, it is shown that hardware acceleration should be considered the preferred solution when there is a need for high performance and energy efficiency. It is however noted that the viability of hardware acceleration is dependent on the required map size and resolution. Finally, it is concluded that no value is offered by hardware acceleration for unconstrained applications where high sampling rates are not required.

<!-- GETTING STARTED -->
## Getting Started

Detailed here is the equipment you need to run the provided code and example experiments

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

### Prerequisites

The following software is required to run the code and experiments.

1. Install the required software needed to run the simulation.

* Ubuntu 22.04

* ROS2 Humble

Install ROS2 Humble following the instructions [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

* [Webots 2022a](https://github.com/cyberbotics/webots/releases/tag/R2022a)

Run the following commands to install Webots 2022a after downloading the .deb file:

```sh
sudo apt install ./webots_2022a_amd64.deb
```

There is a known cirtificate issue with the Webots installation. To fix this, follow the instructions [here](https://github.com/cyberbotics/webots_ros2/issues/465).

* Numpy
```sh
sudo apt-get install python3-numpy
```

2. Build the neccessary ROS2 packages

Clone the repository:

```sh
git clone --recurse-submodules -j8 https://github.com/matthew-william-lock/teensy-vs-upduino-particle-filter
```

Move the ROS2 packages into the `src` folder of your ROS2 workspace:

```sh
cp -r teensy-vs-upduino-particle-filter/src/mcl ~/ros2_ws/src/
cp -r teensy-vs-upduino-particle-filter/src/webots_ros2 ~/ros2_ws/src/
```

Build the ROS2 packages:

```sh
cd ~/ros2_ws
colcon build 
```
> If you encounter an error related to "hardware_interface_DIR", try running the following command:

 ```sh
sudo apt install ros-humble-ros2-control
```

:warning: Please follow the build guide for the ```mcl``` package found in src. This is important and the simulation will not work otherwise as you will have not installed the necessary dependencies.

Source the ROS2 workspace:

```sh
source ~/ros2_ws/install/local_setup.bash
```

<!-- USAGE EXAMPLES -->
## Usage

Show here is documentation of how to run the three experiments that were run for this investigation. Shown in the video below is the pure Python implementation without any embedded localisation (i.e. without the Teensy or Upduino). This base case simulation is extended from [Debby Nirwan](https://github.com/debbynirwan/mcl).

https://user-images.githubusercontent.com/53016036/217649590-610c5c57-9cef-468f-a281-43cfc639612a.mp4

> All of the instructions below assume you have already installed the required software described in the [Getting Started](#getting-started) section.

### Simulated Localisation

Localisation here is perfomed on the host machine using the Python implementation of the particle filter. The following command is used to run the simulation:

```sh
ros2 launch mcl mcl_launch.py rviz:=true mission_time:=2
```

Parameters:
| Parameter                 | Description   |	
| :------------------------ | :-------------|
| rviz                      | Whether to run RViz or not. |
| mission_time              | The time in minutes. |

### Embedded Localisation

Localisation here is perfomed on the Teensy 4.1 microcontroller. Before running the simulation, the code must be uploaded to the Teensy 4.1 and your host machine must be running micro-ROS as described in the [here](https://github.com/matthew-william-lock/teensy-vs-upduino-particle-filter/tree/main/src/teensy_particle_filter).

<img src="/images/experimental_config.png" width="100%">

 The following command is used to run the simulation:

```sh
ros2 launch mcl mcl_teensy_launch.py rviz:=true mission_time:=2
```

Parameters:
| Parameter                 | Description   |	
| :------------------------ | :-------------|
| rviz                      | Whether to run RViz or not. |
| mission_time              | The time in minutes. |

> Acceleration is achieved by using the FPGA accelerator, the experimental setup of which is described [here](). To toggle acceleration, this must be set in the [mcl config](https://github.com/matthew-william-lock/teensy-vs-upduino-particle-filter/blob/main/src/teensy_particle_filter/config/mcl.h) file on the Teensy 4.1. Do not forget to recomplie the code and upload it to the Teensy 4.1. Parameters of the configuration file are described below:

| Parameter                 | Description   |
| :------------------------ | :-------------|
| USE_HARDWARE_ACCELERATION | Whether to use the FPGA accelerator or not |
| NUM_OF_PARTICLES         | The number of particles to use in the particle filter. |

## Miscellanous

Below are some links to other tools and development guides that we found useful during the development of this project.

### Teensy Development
- [VSCode + PlatformIO](https://platformio.org/)
- [micro_ros_arduino_pub_sub_example](https://github.com/botamochi6277/micro_ros_arduino_pub_sub_example)

### Interesting Tools
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
