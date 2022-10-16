# Evaluating the Particle Efficiency Between the Teensy 4.1 and UPduino v3.1 FPGA for Robot Localisation

*Documentation is not finalised*

## Teensy

## FPGA

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

2. When running experiments, disable the random bound by setting ```rb:=false```. See the exampe below

```
https://github.com/matthew-william-lock/ros2-ds
```

## Requirements

- [Ubuntu 22.04]()
- [Numpy]()
- [Webots 2022a](https://github.com/cyberbotics/webots/releases/tag/R2022a)

### Teensy Development
- [VSCode + PlatformIO](https://platformio.org/)
- [micro_ros_arduino_pub_sub_example](https://github.com/botamochi6277/micro_ros_arduino_pub_sub_example)

## Interesting Tools
- [The MRPT project](https://github.com/MRPT/mrpt)
- [OctoMap](https://github.com/OctoMap/octomap)
