# Embedde Particle Filter on Teensy 4.1 using micro-ROS

This repository contains the code for the particle filter running on the Teensy 4.1. Simulation of the robotic platform is done using webots as outlined in the root [Readme](https://github.com/matthew-william-lock/teensy-vs-upduino-particle-filter). Outlined below are the steps to get the particle filter running on the Teensy 4.1.

<img src="/images/experimental_config.png" width="100%">

## Getting Started

### Prerequisites

1. The [platformio VSCode extension](https://platformio.org/install/ide?install=vscode) was used to develop this code. While it is possible to use the command line, we will only outline the steps for the VSCode extension as this is the methodolgy used in our work.

3. micro-ROS

You may install micro-ROS following the [micro-ROS installation guide](https://micro.ros.org/docs/tutorials/core/first_application_linux/) and run the agent following the guide if you would like. For the purpose of the project, we suggest you simply run it within a docker container. 

Please follow the folling [guide](https://docs.docker.com/engine/install/ubuntu/) to install docker.

>If you are facing issues with getting docker to run on Ubuntu 22.04 WSL, please see the following [fix](https://github.com/docker/for-linux/issues/1406#issuecomment-1183487816).

### Building and Uploading the Code

1. Open the ```src/teensy_particle_filter``` folder in VSCode.

2. Build the code by clicking the ```Build``` button in the bottom left corner of the screen or by pressing ```Ctrl+Shift+B```.

In the case you encounter the following build error, 

```
/home/matt/.platformio/packages/toolchain-gccarmnoneeabi/arm-none-eabi/include/c++/5.4.1/bits/random.tcc:1464:42: error: no matching function for call to 'max(float, const double&)'
    _M_d1 = std::round(std::max(1.0, __d1x));
                                          ^
In file included from /home/matt/.platformio/packages/toolchain-gccarmnoneeabi/arm-none-eabi/include/c++/5.4.1/vector:60:0,
                 from src/particleFilter.hpp:5,
                 from src/particleFilter.cpp:1:
/home/matt/.platformio/packages/toolchain-gccarmnoneeabi/arm-none-eabi/include/c++/5.4.1/bits/stl_algobase.h:219:5: note: candidate: template<class _Tp> constexpr const _Tp& std::max(const _Tp&, const _Tp&)
     max(const _Tp& __a, const _Tp& __b)
     ^
/home/matt/.platformio/packages/toolchain-gccarmnoneeabi/arm-none-eabi/include/c++/5.4.1/bits/stl_algobase.h:219:5: note:   template argument deduction/substitution failed:
In file included from /home/matt/.platformio/packages/toolchain-gccarmnoneeabi/arm-none-eabi/include/c++/5.4.1/random:51:0,
                 from src/particle.hpp:5,
                 from src/particleFilter.hpp:7,
                 from src/particleFilter.cpp:1:
/home/matt/.platformio/packages/toolchain-gccarmnoneeabi/arm-none-eabi/include/c++/5.4.1/bits/random.tcc:1464:42: note:   deduced conflicting types for parameter 'const _Tp' ('float' and 'double')
    _M_d1 = std::round(std::max(1.0, __d1x));
                                          ^
In file included from /home/matt/.platformio/packages/toolchain-gccarmnoneeabi/arm-none-eabi/include/c++/5.4.1/vector:60:0,
                 from src/particleFilter.hpp:5,
Compiling .pio/build/teensy41/FrameworkArduino/analog.c.o
                 from src/particleFilter.cpp:1:
/home/matt/.platformio/packages/toolchain-gccarmnoneeabi/arm-none-eabi/include/c++/5.4.1/bits/stl_algobase.h:265:5: note: candidate: template<class _Tp, class _Compare> constexpr const _Tp& std::max(const _Tp&, const _Tp&, _Compare)
     max(const _Tp& __a, const _Tp& __b, _Compare __comp)
     ^
/home/matt/.platformio/packages/toolchain-gccarmnoneeabi/arm-none-eabi/include/c++/5.4.1/bits/stl_algobase.h:265:5: note:   template argument deduction/substitution failed:
In file included from /home/matt/.platformio/packages/toolchain-gccarmnoneeabi/arm-none-eabi/include/c++/5.4.1/random:51:0,
                 from src/particle.hpp:5,
                 from src/particleFilter.hpp:7,
                 from src/particleFilter.cpp:1:
/home/matt/.platformio/packages/toolchain-gccarmnoneeabi/arm-none-eabi/include/c++/5.4.1/bits/random.tcc:1464:42: note:   deduced conflicting types for parameter 'const _Tp' ('float' and 'double')
    _M_d1 = std::round(std::max(1.0, __d1x));
```

go to [bugzilla](https://gcc.gnu.org/bugzilla/attachment.cgi?id=36237&action=edit) and patch ```.platformio/packages/toolchain-gccarmnoneeabi/arm-none-eabi/include/c++/5.4.1/bits/random.tcc```.

> You should see a ```SUCCESS``` message in the terminal.

3. Upload the code by clicking the ```Upload``` button in the bottom left corner of the screen or by pressing ```Ctrl+Shift+U```.

### Running micro-ROS on your host machine

Now that the code is uploaded to the Teensy, we need to run the micro-ROS agent on our host machine to communicate with the Teensy. The Teensy will automatically connect to the micro-ROS agent and start to communicate with the ROS 2 network once the micro-ROS agent is running. 

1. Open a new terminal and run the following command to start the micro-ROS agent using docker.

```
sudo docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO serial --dev /dev/ttyACM0 -v6
```

>Make sure to replace ```/dev/ttyACM0``` with the correct serial port that your Teensy is connected to.

Alternatively, you can install the micro-ROS agent on your host machine by following the instructions [here](https://micro.ros.org/docs/tutorials/core/first_application_linux/) and then run the following command to start the micro-ROS agent.

```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

## Other Notes

### Echo Config String

```bash
ros2 topic echo /particle_filter/config_string --truncate-length 1000 --field data
```

## Useful guides

- [Teensy with Arduino](https://micro.ros.org/docs/tutorials/core/teensy_with_arduino/)
- [micro_ros_platformio](https://github.com/micro-ROS/micro_ros_platformio)
- [micro_ros_arduino](https://github.com/micro-ROS/micro_ros_arduino)
- [How to include a custom ROS message in micro-ROS
](https://micro.ros.org/docs/tutorials/advanced/create_new_type/)
- [Using the Hardware Serial Ports](https://www.pjrc.com/teensy/td_uart.html)