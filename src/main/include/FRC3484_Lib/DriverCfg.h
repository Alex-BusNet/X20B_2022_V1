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
	// #define EN_SWERVE_SCHEME_CRAB
	// #define EN_SWERVE_SCHEME_TWIST
	#define EN_SWERVE_SCHEME_TWITCH
	#define EN_SWERVE_SCHEME_FREE
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
#if defined(DRIVER_SCHEME_XBOX)
	#if defined(EN_SWERVE_CONTROL) || defined(EN_MECANUM_CONTROL)
		#define C_DRIVE_MOMENTUM_Y	XBOX_LS_Y
		#define C_DRIVE_MOMENTUM_X	XBOX_LS_X

		#if defined(EN_SWERVE_SCHEME_FREE) || defined(EN_MECANUM_CONTROL)
			#define C_DRIVE_ROT_LEFT	XBOX_LT		// Left rotation vector
			#define C_DRIVE_ROT_RIGHT	XBOX_RT		// Right rotation vector
			
			#if defined(EN_SWERVE_SCHEME_FREE)
				#define C_DRIVE_COR_Y		XBOX_RS_Y	// Adjust Center of Rotation along longitudinal axis.
				#define C_DRIVE_COR_X		XBOX_RS_X	// Adjust Center of Rotation along latitudinal axis.
			#else // Mecanum drive controls
				#define C_DRIVE_EBRAKE_L	XBOX_LB
				#define C_DRIVE_EBRAKE_R	XBOX_RB
			#endif
		#elif defined(EN_SWERVE_CONTROL)
			#define C_DRIVE_ROT		XBOX_RS_X
		#endif
	#else
		#define C_DRIVE_TANK_LEFT	XBOX_LS_Y
		#define C_DRIVE_TANK_RIGHT	XBOX_RS_Y
	#endif

	#ifndef EN_MECANUM_CONTROL
		#define C_DRIVE_TOGGLE_ROBOT_CENTRIC	XBOX_LB		// Swap between Field-centric/driver-oriented controls and Robot-centric controls.
	#else
		#define C_DRIVE_TOGGLE_ROBOT_CENTRIC	XBOX_BACK
	#endif
#elif defined(DRIVER_SCHEME_LE3D_SINGLE) || defined(DRIVER_SCHEME_LE3D_DUAL)
	// TODO Finish out the controller scheme for swerve, mecanum, tank, arcade, and curvature.
	#define C_DRIVE_MOMENTUM_Y	LE3D_Y
	#define C_DRIVE_MOMENTUM_X	LE3D_X
	#define C_DRIVE_ROT			LE3D_Z

	#ifdef DRIVER_SCHEME_LE3D_DUAL
		#define C_DRIVE_COR_Y		LE3D_Y	// Second joystick Y axis, Adjust Center of Rotation along longitudinal axis.
		#define C_DRIVE_COR_X		LE3D_X	// Second joystick X axis; Adjust Center of Rotation along latitudinal axis.
	#endif

	#define C_DRIVER_TOGGLE_DRIVE_MODE	LE3D_TRIGGER		// Swap between Field-centric/driver-oriented controls and Robot-centric controls.
#elif defined(DRIVER_SCHEME_DS4)
	// TODO Finish out the controller scheme for swerve, mecanum, tank, arcade, and curvature.
	#define C_DRIVER_FWD_REV	DS4_LS_Y
	#define C_DRIVER_SIDE		DS4_LS_X
	#define C_DRIVER_ROT		DS4_RS_X

	#define C_DRIVER_TOGGLE_DRIVE_MODE	DS4_TOUCH		// Swap between Field-centric/driver-oriented controls and Robot-centric controls.
#endif


/* Game Device 1 Controller Map */




#endif // DRIVERCFG_H