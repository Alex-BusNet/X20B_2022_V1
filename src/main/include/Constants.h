// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "units/velocity.h"
#include "units/length.h"
#include "units/constants.h"

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
#define C_FX_LEFT_MASTER			1
#define C_FX_LEFT_SLAVE				2
#define C_FX_RIGHT_MASTER			3
#define C_FX_RIGHT_SLAVE			4
#define C_SPX_INTAKE                5
#define C_SPX_FEED_MASTER           6
#define C_SPX_FEED_SLAVE            7
#define C_FX_LAUNCH_1               8
#define C_FX_LAUNCH_2               9
#define C_SPX_TURRET                10
#define C_SRX_CLIMB                 12 // Usage TBD

/* Other CAN ID's */
#define C_PCM						0
#define C_PDP						11

/* Solenoid Channels */
#define C_DRIVE_SOL					0 // Shifter
#define C_SOL_INTAKE                1
#define C_SOL_LOADER                2
#define C_SOL_LAUNCH_ANGLE          3
#define C_SOL_CLIMB_VERT_UD         4
#define C_SOL_CLIMB_VERT_CLAW       5
#define C_SOL_CLIMB_2ND_STAGE_UD    6
#define C_SOL_CLIMB_2ND_STAGE_CLAW  7

/* Settings */
#define C_DRIVE_DEADBAND			0.05    // 5% Joystick input
#define C_DRIVE_MAX_DEMAND			0.95    // Joystick input scale range (+/-) for normal movements
#define C_DRIVE_MAX_DEMAND_FINE		0.5     // Joystick input scale range (+/-) for fine movements
#define C_DT_WHEEL_TAU				20_ms   // Filter time for encoder feedback
#define C_THROTTLE_SCALE_COEFF		1.5     // Scaling Coefficient for throttle input

/* Drivetrain Parameters */
#define C_X22_TRACK_WIDTH			30.0_in
#define C_X22_DT_WHEEL_DIAM			6.0 // in

#define C_DT_RPM_TO_FPS				(units::constants::pi * C_X22_DT_WHEEL_DIAM) / (60.0 * 12.0)

const double C_GEAR_RATIO_LO		= 1.0 / 18.0;
const double C_GEAR_RATIO_HI		= 1.0 / 9.56;

const double C_DT_MOTOR_MAX_RPM		= 6380.0;
const double C_DT_MOTOR_MAX_RPM_ACT = 6000.0; // TODO: Get max achievable RPM of drivetrain motors.
const double C_DT_ENC_CPR			= 2048.0;

const double C_MAX_GEAR_ENC_LO      = (C_DT_MOTOR_MAX_RPM / 600.0) * (C_DT_ENC_CPR / C_GEAR_RATIO_LO);
const double C_MAX_GEAR_ENC_HI      = (C_DT_MOTOR_MAX_RPM / 600.0) * (C_DT_ENC_CPR / C_GEAR_RATIO_HI);

const double C_DT_SCALE_FACTOR_LO   = ((600.0 * C_GEAR_RATIO_LO) / C_DT_ENC_CPR) * C_DT_RPM_TO_FPS;
const double C_DT_SCALE_FACTOR_HI   = ((600.0 * C_GEAR_RATIO_HI) / C_DT_ENC_CPR) * C_DT_RPM_TO_FPS;

const units::meters_per_second_t C_HI_GEAR_MAX_SPEED 	= 14.9_fps;
const units::meters_per_second_t C_LO_GEAR_MAX_SPEED 	= 7.9_fps;
const units::meters_per_second_t C_SHIFT_UP_SPEED 		= 5.0_fps;
const units::meters_per_second_t C_SHIFT_DOWN_SPEED 	= 3.5_fps;

/* Launcher Parameters */
#define C_LL_ANGLE				20.0 // deg
#define C_LL_HEIGHT				24.0 // inches above floor
const double C_GOAL_HEIGHT	  = 104.0; // inches		


/* Turret Parameters */

/* Intake Parameters */
#define C_INTAKE_DRIVE_SPEED    0.5	// %
#define C_INTAKE_RAMP_TIME		0.5 // Seconds
#define C_FEED_DRIVE_SPEED      0.5 // %

#define C_DI_CH_PROX_SEN        0

/* General Constants */
#define C_SCAN_TIME         10_ms

const std::tuple<int, int> C_BLANK_IDS = std::make_tuple<int, int>(-1, -1);

