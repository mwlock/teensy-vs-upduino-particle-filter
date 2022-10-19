# Particle Filter Implementation on Teensy 4.1 using micro-ROS

# Development Environment 

1. PlatformIO needs ```git```, ```cmake``` and ```pip3``` to handle micro-ROS internal dependencies:

```
sudo apt install -y git cmake python3-pip
```

2. Fixing build errors

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

# Running micro-ROS

```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

# Echo Config String

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