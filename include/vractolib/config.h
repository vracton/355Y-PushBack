#ifndef CONFIG_H
#define CONFIG_H

namespace vconfig {
    const int maxPercent = 90;
    const int maxVel = 1 * maxPercent;
    const int maxVolt = 120 * maxPercent;
    const int deadzone = 10;
    const int updateRate = 10; //ms
}

#endif