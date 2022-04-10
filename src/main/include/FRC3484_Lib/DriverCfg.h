#ifndef DRIVERCFG_H
#define DRIVERCFG_H

/*
	Define driver input mappings and other 
	relevant values here (like input scaling ranges)
*/

/*===============================================================================*/
// Controller input scheme configuration

// Driver Input Scheme Selection
#define DRIVER_SCHEME_XBOX				// Xbox controller
//#define DRIVER_SCHEME_LE3D_SINGLE		// Single Logitech Extreme 3D Joystick
//#define DRIVER_SCHEME_LE3D_DUAL		// Dual Logitech Extreme 3D Joystick
//#define DRIVER_SCHEME_DS4				// Dualshock 4 controller

// Game Device Input Scheme Selection
#define GAMEDEV_SCHEME_XBOX				// Xbox controller
//#define GAMEDEV_SCHEME_LE3D			// Logitech Extreme 3D Joystick
//#define GAMEDEV_SCHEME_DS4			// Dualshock 4 controller
//#define GAMEDEV_SCHEME_BB				// Custom button box input
/*===============================================================================*/

/*===============================================================*/
// Comment and uncomment one of the defines below
// to select the input control scheme that matches 
// robot design.
#define EN_SWERVE_CONTROL
// #define EN_MECANUM_CONTROL
// #define EN_TANK_CONTROL
/*===============================================================*/


/*===============================================================*/
// Enable one of the following defines to switch the drive scheme
// from a pure tank drive (L/R sides controlled separately)
// to one of the kinematic drives.
#ifdef EN_TANK_CONTROL
	// #define EN_ARCADE_CONTROL
	// #define EN_CURVE_CONTROL
#endif
/*===============================================================*/


/*===============================================================*/
// Enable one or more of the following defines to select the 
// allowable swerve drive control schemes. Swerve drive can allow
// for a variety of control schemes which, if programmed correctly,
// can be selected on-the-fly to suit the driver's needs.
#ifdef EN_SWERVE_CONTROL
	// #define EN_SWERVE_SCHEME_SNAKE
	#define EN_SWERVE_SCHEME_CRAB
	#define EN_SWERVE_SCHEME_TWIST
	// #define EN_SWERVE_SCHEME_TWITCH
	// #define EN_SWERVE_SCHEME_FREE	// Free-form mode can't be done with one joystick
#endif
/*===============================================================*/


/* USB Port Mapping */
#ifndef DRIVER_SCHEME_LE3D_DUAL
	#define DRIVER_USB	0
	#define GAMEDEV_USB	1
#else
	#define DRIVER_USB_LEFT		0
	#define DRIVER_USB_RIGHT	1
	#define GAMEDEV_USB			2
#endif

/* Driver Controller Map */
#if defined(EN_SWERVE_CONTROL) || defined(EN_MECANUM_CONTROL)
	#if defined(DRIVER_SCHEME_XBOX)
		#define C_DRIVE_MOMENTUM_Y	XBOX_LS_Y
		#define C_DRIVE_MOMENTUM_X	XBOX_LS_X
	#elif defined(DRIVER_SCHEME_DS4)
		#define C_DRIVE_MOMENTUM_Y	DS4_LS_Y
		#define C_DRIVE_MOMENTUM_X	DS4_LS_X
	#elif defined(DRIVER_SCHEME_LE3D_SINGLE) || defined(DRIVER_SCHEME_LE3D_DUAL)
		#define C_DRIVE_MOMENTUM_Y	LE3D_Y	// Dual - Left joystick
		#define C_DRIVE_MOMENTUM_X	LE3D_X	// Dual - Left joystick
	#endif

	#if (defined(EN_SWERVE_SCHEME_CRAB) && defined(EN_SWERVE_SCHEME_TWIST)) || defined(EN_MECANUM_CONTROL)
		#if defined(DRIVER_SCHEME_XBOX)
			#define C_DRIVE_ROT		XBOX_RS_X
		#elif defined(DRIVER_SCHEME_DS4)
			#define C_DRIVE_ROT		DS4_RS_X
		#elif defined(DRIVER_SCHEME_LE3D_SINGLE)
			#define C_DRIVE_ROT		LE3D_TWIST
		#elif defined(DRIVER_SCHEME_LE3D_DUAL) && defined(EN_MECANUM_CONTROL)
			#define C_DRIVE_ROT		LE3D_X	// Right joystick
		#endif
	#elif defined(EN_SWERVE_SCHEME_FREE)
		#if defined(DRIVER_SCHEME_XBOX)
			#define C_DRIVE_ROT_LEFT	XBOX_LT		// Left rotation vector
			#define C_DRIVE_ROT_RIGHT	XBOX_RT		// Right rotation vector

			#define C_DRIVE_COR_Y		XBOX_RS_Y	// Adjust Center of Rotation along longitudinal axis.
			#define C_DRIVE_COR_X		XBOX_RS_X	// Adjust Center of Rotation along latitudinal axis.
		#elif defined(DRIVER_SCHEME_DS4)
			#define C_DRIVE_ROT_LEFT	DS4_L2		// Left rotation vector
			#define C_DRIVE_ROT_RIGHT	DS4_R2		// Right rotation vector

			#define C_DRIVE_COR_Y		DS4_RS_Y	// Adjust Center of Rotation along longitudinal axis.
			#define C_DRIVE_COR_X		DS4_RS_X	// Adjust Center of Rotation along latitudinal axis.
		#elif defined(DRIVER_SCHEME_LE3D_DUAL)
			#define C_DRIVE_ROT			LE3D_TWIST	// Right joystick twist

			#define C_DRIVE_COR_Y		LE3D_Y		// Right joystick; Adjust Center of Rotation along longitudinal axis.
			#define C_DRIVE_COR_X		LE3D_X		// Right joystick; Adjust Center of Rotation along latitudinal axis.
		#endif
			#define C_DRIVE_ROT_LEFT	XBOX_LT		// Left rotation vector
			#define C_DRIVE_ROT_RIGHT	XBOX_RT		// Right rotation vector
	#endif

	#ifdef EN_MECANUM_CONTROL
		#if defined(DRIVER_SCHEME_XBOX)
			#define C_DRIVE_EBRAKE_L	XBOX_LB
			#define C_DRIVE_EBRAKE_R	XBOX_RB
		#elif defined(DRIVER_SCHEME_DS4)
			#define C_DRIVE_EBRAKE_L	DS4_L1
			#define C_DRIVE_EBRAKE_R	DS4_R1
		#elif defined(DRIVER_SCHEME_LE3D_SINGLE)
			#define C_DRIVE_EBRAKE_L	LE3D_BTN_7	// verify this
			#define C_DRIVE_EBRAKE_R	LE3D_BTN_10	// verify this
		#endif
	#endif
#elif defined(EN_TANK_CONTROL)
	#if defined(EN_ARCADE_CONTROL)
		#if  defined(DRIVER_SCHEME_XBOX)
			#define C_DRIVE_THROTTLE_AXIS	XBOX_LS_Y
			#define C_DRIVE_STEER_AXIS		XBOX_LS_X
		#elif defined(DRIVER_SCHEME_DS4)
			#define C_DRIVE_THROTTLE_AXIS	DS4_LS_Y
			#define C_DRIVE_STEER_AXIS		DS4_LS_X
		#elif defined(DRIVE_SCHEME_LE3D_SINGLE)
			#define C_DRIVE_THROTTLE_AXIS	LE3D_Y
			#define C_DRIVE_STEER_AXIS		LE3D_X
		#endif
	#elif defined(EN_CURVE_CONTROL)
		#if  defined(DRIVER_SCHEME_XBOX)
			#define C_DRIVE_THROTTLE_AXIS	XBOX_LS_Y
			#define C_DRIVE_STEER_AXIS		XBOX_RS_X
		#elif defined(DRIVER_SCHEME_DS4)
			#define C_DRIVE_THROTTLE_AXIS	DS4_LS_Y
			#define C_DRIVE_STEER_AXIS		DS4_RS_X
		#elif defined(DRIVE_SCHEME_LE3D_SINGLE)
			#define C_DRIVE_THROTTLE_AXIS	LE3D_Y
			#define C_DRIVE_STEER_AXIS		LE3D_X
		#endif
	#else
		#if defined(DRIVER_SCHEME_XBOX)
			#define C_DRIVE_TANK_LEFT	XBOX_LS_Y
			#define C_DRIVE_TANK_RIGHT	XBOX_RS_Y
		#elif defined(DRIVER_SCHEME_DS4)
			#define C_DRIVE_TANK_LEFT	DS4_LS_Y
			#define C_DRIVE_TANK_RIGHT	DS4_RS_Y
		#elif defined(DRIVER_SCHEME_LE3D_DUAL)
			#define C_DRIVE_TANK_LEFT	LE3D_Y 	// Left joystick
			#define C_DRIVE_TANK_RIGHT	LE3D_Y	// Right joystick
		#endif
	#endif
#endif



/* Game Device 1 Controller Map */




#endif // DRIVERCFG_H