#ifndef X22_LAUNCHER_H
#define X22_LAUNCHER_H

#include "FRC3484_Lib/components/SC_Limelight.h"
#include "FRC3484_Lib/components/SC_ColorSensor.h"
#include "FRC3484_Lib/components/SC_PID.h"

#include "FRC3484_Lib/utils/SC_Functions.h"
#include "FRC3484_Lib/utils/SC_Datatypes.h"

#include "ctre/phoenix/motorcontrol/can/TalonFX.h"
#include "ctre/phoenix/motorcontrol/can/VictorSPX.h"

#include "frc/Solenoid.h"
#include "frc/DigitalInput.h"

class X22_Launcher
{
public:
	X22_Launcher(int CenterID, int OuterID, int TurretID,
				 SC::SC_Solenoid Loader, SC::SC_Solenoid AngleCtrl, 
				 frc::I2C::Port ColorSenPort);
	~X22_Launcher();

	void Auto(bool Run);

	void SetCenterPIDTune(SC::SC_PIDConstants PIDC);
	void SetOuterPIDTune(SC::SC_PIDConstants PIDC);



private:
	bool _HasCargoInLoader();

	void _SpoolFlywheel();
	
	ctre::phoenix::motorcontrol::can::TalonFX *_launch_motor_center, *_launch_motor_outer;
	ctre::phoenix::motorcontrol::can::VictorSPX *_turret_motor;
	
	frc::Solenoid *_sol_loader, *_sol_angle;

	SC::SC_ColorSensor *_sen_loader;
	SC::SC_Limelight *_vision;

	SC::SC_PID *_center_PID, *_outer_PID;
};


#endif // X22_LAUNCHER_H