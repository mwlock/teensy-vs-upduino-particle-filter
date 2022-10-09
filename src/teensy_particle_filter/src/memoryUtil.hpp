// Guards
#ifndef MEMORYUTIL_HPP
#define MEMORYUTIL_HPP

#include <Arduino.h>

#include <cstdint>
#include <tuple>

// Tools developed from sources
// https://forum.pjrc.com/threads/46506-3-6-Available-Memory
// https://forum.pjrc.com/threads/33443-How-to-display-free-ram/page2
// https://forum.pjrc.com/threads/57326-T4-0-Memory-trying-to-make-sense-of-the-different-regions


// Class to measure heap memory usage on Teensy 4.1
class MemoryUtil
{
public:
    
    static std::tuple<int, int, int> memInfo();
    static FLASHMEM void getFreeITCM();
};

#endif