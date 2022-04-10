// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "units/velocity.h"
#include "units/length.h"
#include "units/constants.h"

#include "FRC3484_Lib/utils/SC_ControllerMaps.h"

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

#define C_DISABLED_CHANNEL			-1	// Device or channel is not used.

/*==========*/
/* CAN ID's */
/*==========*/
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
#define C_SRX_CLIMB                 12
#define C_SPX_CLIMB                 C_DISABLED_CHANNEL //15 
#define C_PCM						14
#define C_PDP						11


/*===================*/
/* Solenoid Channels */
/*===================*/
#define C_DRIVE_SOL					0 // Shifter
#define C_SOL_INTAKE                1
#define C_SOL_LOADER                2
#define C_SOL_LAUNCH_ANGLE          3
#define C_SOL_CLIMB_VERT_UD         4
#define C_SOL_CLIMB_VERT_CLAW       5
#define C_SOL_CLIMB_2ND_STAGE_UD    6
#define C_SOL_CLIMB_2ND_STAGE_CLAW  7


/*==============*/
/* DIO Channels */
/*==============*/
#define C_DI_CH_STORED_SW			0
#define C_DI_CH_TURRET_LS_MIN		1
#define C_DI_CH_TURRET_LS_MAX		2
#define C_DI_CH_LOADER_DOWN_SW		3


/*==========*/
/* Settings */
/*==========*/
#define C_DRIVE_DEADBAND			0.05    // 5% Joystick input
#define C_DRIVE_MAX_DEMAND			0.95    // Joystick input scale range (+/-) for normal movements
#define C_DRIVE_MAX_DEMAND_FINE		0.5     // Joystick input scale range (+/-) for fine movements
#define C_DT_WHEEL_TAU				20_ms   // Filter time for encoder feedback
#define C_THROTTLE_SCALE_COEFF		1.5     // Scaling Coefficient for throttle input


/*=======================*/
/* Drivetrain Parameters */
/*=======================*/
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

const units::feet_per_second_t C_HI_GEAR_MAX_SPEED 	= 14.9_fps;
const units::feet_per_second_t C_LO_GEAR_MAX_SPEED 	= 7.9_fps;
const units::feet_per_second_t C_SHIFT_UP_SPEED     = 5.0_fps;
const units::feet_per_second_t C_SHIFT_DOWN_SPEED 	= 3.5_fps;


/*=====================*/
/* Launcher Parameters */
/*=====================*/
#define C_LL_ANGLE					20.0 // deg
#define C_LL_HEIGHT					24.0 // inches above floor

#define C_LL_ANGLE_ALT 				0.0
#define C_LL_HEIGHT_ALT				0.0
const double C_GOAL_HEIGHT        = 104.0; //inches

#define C_LAUNCH_MAX_DEMAND_PCT			0.6 //0.00 // % Output; Max motor output percentage we will allow the system to achieve
#define C_LAUNCH_WHEEL_DIAM         	4.0 // in
const double C_LAUNCH_RPM_TO_FPS        = (units::constants::pi * C_LAUNCH_WHEEL_DIAM) / (60.0 * 12.0);

const double C_LAUNCH_GEAR_RATIO    	= 1.0;
const double C_LAUNCH_SCALE_FACTOR_RPM 	= ((600.0 * C_LAUNCH_GEAR_RATIO) / C_DT_ENC_CPR);
const double C_LAUNCH_SCALE_FACTOR_FPS	= C_LAUNCH_SCALE_FACTOR_RPM * C_LAUNCH_RPM_TO_FPS;

const double C_LAUNCH_MAX_WHEEL_VEL_RPM = 6000.0 * C_LAUNCH_GEAR_RATIO * C_LAUNCH_MAX_DEMAND_PCT;
const double C_LAUNCH_MAX_WHEEL_VEL_FPS	= C_LAUNCH_MAX_WHEEL_VEL_RPM * C_LAUNCH_RPM_TO_FPS; // Feet per second

// Values used in logic
const double C_LAUNCH_MAX_WHEEL_VEL 	= C_LAUNCH_MAX_WHEEL_VEL_RPM;
const double C_LAUNCH_SCALE_FACTOR		= C_LAUNCH_SCALE_FACTOR_RPM;

// Manual launch values
const double C_MANUAL_LAUNCH_VEL_CEN 	= 0.70 * C_LAUNCH_MAX_WHEEL_VEL; // 0.7 center wheel manual launch velocity
const double C_MANUAL_LAUNCH_VEL_OUT 	= 0.65 * C_LAUNCH_MAX_WHEEL_VEL; // 0.65 outer wheel manual launch velocity

const double C_LONG_LAUNCH_VEL_CEN		= 0.80 * C_LAUNCH_MAX_WHEEL_VEL; //0.9
const double C_LONG_LAUNCH_VEL_OUT		= 0.75 * C_LAUNCH_MAX_WHEEL_VEL; //0.85

const double C_EJECT_VEL_CEN			= 0.40 * C_LAUNCH_MAX_WHEEL_VEL;
const double C_EJECT_VEL_OUT			= 0.40 * C_LAUNCH_MAX_WHEEL_VEL;

// Tuning values
#define C_LAUNCH_CEN_DEFAULT_KP		1.0 //90.0
#define C_LAUNCH_CEN_DEFAULT_KI		2.5 //1.75 //60.0

#define C_LAUNCH_OUT_DEFAULT_KP		1.0 //90.0
#define C_LAUNCH_OUT_DEFAULT_KI		1.75 //60.0

// Control parameters
#define C_LAUNCH_ANGLE_THRESHOLD	9.5 * 12 //9.5ft
#define C_LAUNCH_CEN_READY_BAND		1.0 // % Output
#define C_LAUNCH_OUT_READY_BAND		1.0 //% Output; probably have a larger tolernce to the outer wheel speed comepared to the inner

#define C_CARGO_LOADED_THRESH		130.0	// Units

// Conversion values


/*===================*/
/* Turret Parameters */
/*===================*/
#define C_TURRET_DEFAULT_SEEK_VEL	0.30 // % Output
#define C_TURRET_DEFAULT_KP			0.20
#define C_TURRET_DEFAULT_KI			0.10

#define C_TURRET_READY_BAND			5.0  //UNITS?

#define C_TURRET_MAX_DEMAND			0.25 // % Output


/*===================*/
/* Intake Parameters */
/*===================*/
#define C_INTAKE_DRIVE_SPEED		1.0 // *100%
#define C_INTAKE_RAMP_TIME			0.50 // Seconds
#define C_FEED_DRIVE_SPEED			1.00 // *100%


/*======================*/
/* CONTROLLER CONSTANTS */
/*======================*/
#define C_DRIVER_USB                 0
#define C_GAMEDEV_USB                1

// Game Device control input scheme 
// #define GD_SCHEME_JOYSTICK	/* Logitech Extreme-3D Pro Joystick Scheme */
#define GD_SCHEME_XBOX			/* Xbox Controller Scheme */
//#define GD_SCHEME_DS4 		/* DualShock 4 Controller Scheme */


//Climber controls are on separate controller.
#define CLIMB_CONTROL_SEPARATE 
#ifdef CLIMB_CONTROL_SEPARATE
#define C_CLIMB_USB                 2
#endif

//Climb input are 1 button per state. if commented out, buttom are treated as 
//controlling a stage's extaed/release or a stage's claw
// #define CLIMB_INPUTS_AS_SEQENCE

/**
 * Set The Driver mode
 */
//#define DRIVE_MODE_TANK
#define DRIVE_MODE_ARCADE
//#define DRIVE_MODE_CURVE

#if defined(DRIVE_MODE_TANK)
	#define C_DRIVER_LEFT_AXIS			XBOX_LS_Y
	#define C_DRIVER_RIGHT_AXIS			XBOX_RS_Y
#elif defined(DRIVE_MODE_ARCADE)
	#define C_DRIVER_THROTTLE_AXIS		XBOX_LS_Y 
	#define C_DRIVER_STEER_AXIS			XBOX_RS_X //XBOX_LS_X (arcade)
#elif defined(DRIVE_MODE_CURVE)
	#define C_DRIVER_THROTTLE_AXIS		XBOX_LS_Y
	#define C_DRIVER_STEER_AXIS			XBOX_LS_X
#endif

#define C_DRIVER_SHIFT_LOW_BTN        	XBOX_A
#define C_DRIVER_EBRAKE					XBOX_LB

#define DRIVER_FINE_ADJ_MODE
#ifdef DRIVER_FINE_ADJ_MODE
#define C_DRIVE_ADJ_BTN               	XBOX_RB
#endif

/*===================*/
/* Game Device Input */
/*===================*/
#ifdef GD_SCHEME_XBOX
	#define C_GD_INTAKE					XBOX_A
	#define C_GD_LAUNCH_AUTO			XBOX_B
	#define C_GD_FORCE_FEED				XBOX_X
	#define C_GD_FEED_EJECT				XBOX_Y
	#define C_GD_LAUNCH_LONG			XBOX_LB
	#define C_GD_LAUNCH_EJECT			XBOX_BACK
	#define C_GD_LAUCH_SHORT			XBOX_RB

	//Turret override axis
	#define C_GD_TURRET_OR_AXIS			XBOX_LS_X
#elif defined(GD_SCHEME_JOYSTICK)
	#define C_GD_INTAKE					LE3D_BTN_5
	#define C_GD_LAUNCH_AUTO			LE3D_TRIGGER
	#define C_GD_FORCE_FEED				LE3D_BTN_8
	#define C_GD_FEED_EJECT				LE3D_HAT_DOWN
	#define C_GD_LAUNCH_LONG			LE3D_BTN_2
	#define C_GD_LAUNCH_EJECT			LE3D_HAT_UP

	//Turret override axis
	#define C_GD_TURRET_OR_AXIS			LE3D_X
#elif defined(GD_SCHEME_DS4)
	#define C_GD_INTAKE					DS4_CROSS
	#define C_GD_LAUNCH_AUTO			DS4_CIRCLE
	#define C_GD_FORCE_FEED				DS4_SQUARE
	#define C_GD_FEED_EJECT				DS4_TRIANGLE
	#define C_GD_LAUNCH_LONG			DS4_L1
	#define C_GD_LAUNCH_EJECT			DS4_R1

	//Turret override axis
	#define C_GD_TURRET_OR_AXIS			DS4_LS_X
#endif


/*climber input*/
#ifdef CLIMB_CONTROL_SEPARATE
	#ifndef  CLIMB_INPUTS_AS_SEQUENCE
		#define C_CLIMB_STAGE1_EXTEND 	LE3D_BTN_3
		#define C_CLIMB_STAGE1_CLAW		LE3D_BTN_5
		#define C_CLIMB_STAGE2_EXTEND	LE3D_BTN_4
		#define C_CLIMB_STAGE2_CLAW		LE3D_BTN_6
	#else 
		#define C_CLIMB_STATE_1       0
		#define C_CLIMB_STATE_2       1
		#define C_CLIMB_STATE_3       2
		#define C_CLIMB_STATE_4       3
		#define C_CLIMB_STATE_5       4
		#define C_CLIMB_STATE_6       5
		#define C_CLIMB_STATE_7       6
		#define C_CLIMB_STATE_8       7
	#endif
#else
	#ifndef CLIMB_INPUTS_AS_SEQUENCE
		#define C_CLIMB_STAGE1_EXTEND 7
		#define C_CLIMB_STAGE1_CLAW   8
		#define C_CLIMB_STAGE2_EXTEND 9
		#define C_CLIMB_STAGE2_CLAW   10
	#else
		#define C_CLIMB_STAGE_1 7
		#define C_CLIMB_STAGE_2 8
		#define C_CLIMB_STAGE_3 9
		#define C_CLIMB_STAGE_4 10
		#define C_CLIMB_STAGE_5 11
		#define C_CLIMB_STAGE_6 12
		#define C_CLIMB_STAGE_7 13
		#define C_CLIMB_STAGE_8 14
	#endif
#endif


/*==================*/
/* Debouncer Config */
/*==================*/
// Driver inputs
#define C_MANUSHIFT_DBNC_TIME		0.100_s	// Force-low gear shift control input debounce time
#define C_FINEADJ_DBNC_TIME			0.100_s	// Fine-Adjustment controller input debounce time

// Game device inputs
#define C_INTAKE_BTN_DBNC_TIME		0.100_s	// Intake deploy controller input debounce time
#define C_FEEDEJECT_DBNC_TIME		0.100_s	// Feeder Eject controller input debounce time
#define C_MANUFEED_DBNC_TIME		0.100_s // Maunal feed controller input debounce time

#define C_AUTOLAUNCH_DBNC_TIME		0.100_s	// Auto-launch controller input debounce time
#define C_MANULAUNCH_DBNC_TIME		0.100_s // Manual launch controller input debounce time
#define C_MANUEJECT_DBNC_TIME		0.100_s	// Manaul launcher eject input debounce time

// Climber inputs
#define C_CLIMB_DBNC_TIME			0.250_s	// Climb controller input debounce time

// Auto-functions
#define C_AUTOSHIFT_DBNC_TIME		0.250_s	// Auto-shift debounce time

#define C_CARGOSTORED_DBNC_TIME		0.050_s // `Cargo stored` switch debounce time.
#define C_CARGOLOADED_DBNC_TIME		0.250_s // `Cargo loaded` status debounce time.

#define C_TURRET_MAX_DBNC_TIME		0.050_s	// Turret max travel limit switch debounce time
#define C_TURRET_MIN_DBNC_TIME		0.050_s // Turret min travel limit switch debounce time
#define C_TURRET_READY_DBNC_TIME	0.100_s // Turret ready debounce time

#define C_LOADER_SW_DBNC_TIME		0.100_s // Loader down switch debounce time

#define C_CEN_READY_DBNC_TIME		0.100_s // Center launch wheel ready debounce time
#define C_OUT_READY_DBNC_TIME		0.100_s // Outer launch wheel ready debounce time

#define C_ALT_ANGLE_DBNC_TIME		0.250_s	// Alternate launch angle debounce time


/*=====================*/
/* Action Delay Config */
/*=====================*/
#define C_INTAKEMOTOR_DELAY_TIME	1.0_s	// Time after intake is deployed before the motor starts running
#define C_AUTOFEED_OFF_DELAY_TIME	1.0_s	// Time after `cargo stored` switch is released before stopping the feed motors.
#define C_LOADED_OFF_DELAY_TIME		1.0_s // Time after color sensor detects there is no cargo before changing the status.

#define C_AUTO_TAXI_TIME			1.0_s

/*===================*/
/* General Constants */
/*===================*/
#define C_SCAN_TIME					0.020_s
#define C_SCAN_TIME_SEC				C_SCAN_TIME.value() // Seconds

const std::tuple<int, int> C_BLANK_IDS = std::make_tuple<int, int>(C_DISABLED_CHANNEL, C_DISABLED_CHANNEL);


