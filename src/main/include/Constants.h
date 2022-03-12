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

#define C_MANUAL_LAUNCH_VEL_CEN 0.60 // Center launch wheel motor %
#define C_MANUAL_LAUNCH_VEL_OUT 0.30 // Outer launch wheel motor %

#define C_EJECT_VEL_CEN 0.20 // Center launch wheel ejection motor %
#define C_EJECT_VEL_OUT 0.40 // Outer launch wheel ejection motor %

#define C_LAUNCH_CEN_DEFAULT_KP	0.40
#define C_LAUNCH_CEN_DEFAULT_KI 0.20

#define C_LAUNCH_OUT_DEFAULT_KP 0.40
#define C_LAUNCH_OUT_DEFAULT_KI 0.20

#define C_LAUNCH_ANGLE_THRESHOLD 	15*12 // 15 ft (?)
#define C_LAUNCH_CEN_READY_BAND		0.1 // % Output
#define C_LAUNCH_OUT_READY_BAND		0.15	// % Output; probably have a larger tolernce to the outer wheel speed compared to the inner

/* Turret Parameters */
#define C_DI_CH_TURRET_LS_MIN  -1 //1
#define C_DI_CH_TURRET_LS_MAX  -1 //2

#define C_TURRET_DEFAULT_SEEK_VEL		0.5 // % Output
#define C_TURRET_DEFAULT_KP		0.2
#define C_TURRET_DEFAULT_KI		0.1

#define C_TURRET_READY_BAND		5.0 // Units?

/* Intake Parameters */
#define C_INTAKE_DRIVE_SPEED    0.5	// %
#define C_INTAKE_RAMP_TIME		0.5 // Seconds
#define C_FEED_DRIVE_SPEED      0.5 // %

#define C_DI_CH_PROX_SEN        -1 //0

/*======================*/
/* Controller Constants */
/*======================*/
#define C_DRIVER_USB 0
#define C_GAMEDEV_USB 1

// Climber controls are on separate controller.
#define CLIMB_CONTROL_SEPARATE
#ifdef CLIMB_CONTROL_SEPARATE
#define C_CLIMB_USB 2
#endif

// Climb input are 1 button per state. If commented out, buttons are treated as 
// controlling a stage's extend/release or a stage's claw.
#define CLIMB_INPUTS_AS_SEQUENCE

/*
 * Set the driver mode
 */
// #define DRIVE_MODE_TANK
#define DRIVE_MODE_ARCADE
// #define DRIVE_MODE_ARCADE

#ifdef DRIVE_MODE_TANK
#define C_DRIVER_LEFT_AXIS      1	// LS Y
#define C_DRIVER_RIGHT_AXIS     3	// RS Y
#elif defined(DRIVE_MODE_ARCADE)
#define C_DRIVER_THROTTLE_AXIS  1	// LS Y
#define C_DRIVER_STEER_AXIS     4	// RS X
#elif defined(DRIVE_MODE_ARCADE)
#define C_DRIVER_THROTTLE_AXIS  1	// LS Y
#define C_DRIVER_STEER_AXIS     2	// LS X
#endif

#define C_DRIVER_SHIFT_LOW_BTN	1	// A

#define DRIVER_FINE_ADJ_MODE
#ifdef DRIVER_FINE_ADJ_MODE
#define C_DRIVE_ADJ_BTN		6	// RB
#endif

/* Game Device Inputs */
#define C_GD_INTAKE			1
#define C_GD_LAUNCH_AUTO	2
#define C_GD_FORCE_FEED		3
#define C_GD_FEED_EJECT		4
#define C_GD_LAUNCH_FORCE	5
#define C_GD_LAUNCH_EJECT	6

// Turret override axis
#define C_GD_TURRET_OR_AXIS	1

/* Climber inputs */
#ifdef CLIMB_CONTROL_SEPARATE
	#ifndef CLIMB_INPUTS_AS_SEQUENCE
		#define C_CLIMB_STAGE1_EXTEND   1
		#define C_CLIMB_STAGE1_CLAW		2
		#define C_CLIMB_STAGE2_EXTEND	3
		#define C_CLIMB_STAGE2_CLAW		4
	#else
		#define C_CLIMB_STATE_1	0
		#define C_CLIMB_STATE_2 1
		#define C_CLIMB_STATE_3 2
		#define C_CLIMB_STATE_4 3
		#define C_CLIMB_STATE_5 4
		#define C_CLIMB_STATE_6 5
		#define C_CLIMB_STATE_7 6
		#define C_CLIMB_STATE_8 7
	#endif
#else
	#ifndef CLIMB_INPUTS_AS_SEQUENCE
		#define C_CLIMB_STAGE1_EXTEND   7
		#define C_CLIMB_STAGE1_CLAW		8
		#define C_CLIMB_STAGE2_EXTEND	9
		#define C_CLIMB_STAGE2_CLAW		10
	#else
		#define C_CLIMB_STATE_1	7
		#define C_CLIMB_STATE_2 8
		#define C_CLIMB_STATE_3 9
		#define C_CLIMB_STATE_4 10
		#define C_CLIMB_STATE_5 11
		#define C_CLIMB_STATE_6 12
		#define C_CLIMB_STATE_7 13
		#define C_CLIMB_STATE_8 14
	#endif
#endif


/* General Constants */
#define C_SCAN_TIME         0.010_s // seconds

const std::tuple<int, int> C_BLANK_IDS = std::make_tuple<int, int>(-1, -1);

static double C_LAUNCHER_CURVE_X[10] = 
{
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0
};

static double C_LAUNCHER_CEN_CURVE_Y[10] = 
{
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0
};

static double C_LAUNCHER_OUT_CURVE_Y[10] = 
{
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0
};
