#ifndef GLOBALS_H
#define GLOBALS_H

/*
    This is for objects that are to be created once and used by multiple objects.

    There should be very few objects declared here. DO NOT USE for declaring robot
    subsystems, components, or similar objects.
 */

#include "FRC3484_Lib/utils/SC_Shuffleboard.h"

static SC::SC_SBBuilder *DebugDash = new SC::SC_SBBuilder("X22 - Debug");
static SC::SC_SBBuilder *DriverDash = new SC::SC_SBBuilder("X22 - Driver");

#endif // GLOBALS_H