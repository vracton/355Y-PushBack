#ifndef CONFIG_H
#define CONFIG_H

namespace vconfig {
    const int maxPercent = 85;
    const int maxVel = 1.2 * maxPercent;
    const int maxVolt = 120 * maxPercent;
    const int forwardDeadzone = 10;
    const int turnDeadzone = 5;
    const int updateRate = 10; //ms
}

#endif