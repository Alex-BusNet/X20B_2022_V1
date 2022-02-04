#ifndef SC_CONSTANTS_H
#define SC_CONSTANTS_H

#include "units/angle.h"
#include "units/time.h"

#define null nullptr

namespace SC
{
    // 360 degrees (1 revolution) in radians. Useful for converting angluar movement to tangential movement.
    static const units::angle::radian_t C_REVOLUTION_RADS = 360_deg;

    // Time between calls of Periodic() by the robot.
    static const units::time::second_t C_CYCLE_TIME = 10_ms;

}
#endif