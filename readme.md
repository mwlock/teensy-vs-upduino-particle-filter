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
