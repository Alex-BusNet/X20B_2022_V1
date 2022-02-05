// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "units/velocity.h"
/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

/* Motor controller CAN ID's */
#define C_SRX_FL_CIM  6
#define C_SRX_FL_MINI 7
#define C_SRX_FR_CIM  12
#define C_SRX_FR_MINI 13
#define C_SRX_BL_CIM  8
#define C_SRX_BL_MINI 9
#define C_SRX_BR_CIM  10
#define C_SRX_BR_MINI 11

#define C_FX_LAUNCH    4     // Launcher Master motor
#define C_FC_LAUNCH_S  3     // Launcher Slave motor
#define C_FX_INTAKE    0
#define C_FX_SHIMMY   -1    // Not used on X20B for non-comp use

#define C_SPX_TURRET   2
#define C_SPX_SHIELD   5
#define C_SPX_SPARE_1 -1
#define C_SPX_SPARE_2 -1

/* Other CAN ID's */
#define C_PCM         1
#define C_PDP         14

/* Solenoid Channels */
#define C_DRIVE_SOL   0 // Shifter
#define C_INTAKE_SOL  1 // drop the intake
#define C_LOADER_SOL  2 // Drive the loader
#define C_WINCH_SOL   3 // Engage the winch motor 
#define C_HANG_SOL    4 // Extend the hang device

/* Datatype Shortcuts */
#define rpm_t units::revolutions_per_minute_t

/* Settings */
#define C_DRIVE_DEADBAND 0.05

const units::feet_per_second_t C_HIGH_GEAR_MAX_SPEED = 15_fps;
const units::feet_per_second_t C_LOW_GEAR_MAX_SPEED = 10_fps;

const std::tuple<int, int> C_BLANK_IDS = std::make_tuple<int, int>(-1, -1);

