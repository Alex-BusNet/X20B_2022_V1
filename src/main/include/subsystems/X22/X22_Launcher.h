#ifndef X22_LAUNCHER_H
#define X22_LAUNCHER_H

#include "FRC3484_Lib/components/SC_Limelight.h"
#include "FRC3484_Lib/components/SC_PID.h"
#include "FRC3484_Lib/utils/SC_Shuffleboard.h"

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
	 			 int TurretLSMin_Ch, int TurretLSMax_Ch, 
				 SC::SC_Solenoid Loader, SC::SC_Solenoid AngleCtrl);
	~X22_Launcher();

	/**
	 * @brief Runs through the automated targeting and launch sequence
	 */
	void Periodic(bool Auto, double TurretVel, bool Launch, bool Eject);



	void SetCenterPIDTune(SC::SC_PIDConstants PIDC);
	void SetOuterPIDTune(SC::SC_PIDConstants PIDC);

	/**
	 * @brief Returns whether the turret's minimum travel limit switch is pressed. Returns false if the input is null.
	 */
	bool IsTurretAtMin();
	
	/**
	 * @brief Returns whether the turret's maximum travel limit switch is pressed. Returns false if the input is null.
	 */
	bool IsTurretAtMax();

private:
	void _TrackTarget(bool Enable);

	void _SpoolFlywheel(bool Enable);

	/**
	 * @brief Provides manual control of launcher/turret mechanism.
	 * 
	 * @param TurretVel Driver input to move the launcher left/right.
	 * @param Launch Launcher spools up to a predefined speed and then laucnhes the ball.
	 * @param Eject Launcher spools up to a slow, predefined speed and then launches the ball.
	 */
	void _Manual(double TurretVel, bool Launch, bool Eject);


	void _InitDashboard();
	void _UpdateDashboard();

	bool _turretReady, _center_wheel_ready, _outer_wheel_ready;
	int _turretSearchDir;
	double _turretErr, _turret_P, _turret_I, _turret_CV;
	double _launch_dist;
	
	ctre::phoenix::motorcontrol::can::TalonFX *_launch_motor_center, *_launch_motor_outer;
	ctre::phoenix::motorcontrol::can::VictorSPX *_turret_motor;
	
	frc::DigitalInput *_ls_turret_travel_min, *_ls_turret_travel_max;

	frc::Solenoid *_sol_loader, *_sol_angle;

	SC::SC_Limelight *_vision;

	SC::SC_PID *_center_PID, *_outer_PID;

	// Dashboard outputs
	SC::SC_SBItem<double> *_nt_turret_err;
	SC::SC_SBItem<bool> *_nt_turret_ls_min, *_nt_turret_ls_max, *_nt_turret_ready;
	SC::SC_SBItem<double> *_nt_launcher_vel_sp, *_nt_launcher_vel_pv;

	// Dashboard inputs
	SC::SC_SBItem<double> *_nt_turret_kp, *_nt_turret_ki;
	SC::SC_SBItem<double> *_nt_launcher_kp, *_nt_launcher_ki;


	double C_LAUNCHER_CURVE_X[10] = 
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

	double C_LAUNCHER_CEN_CURVE_Y[10] = 
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

	double C_LAUNCHER_OUT_CURVE_Y[10] = 
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

};


#endif // X22_LAUNCHER_H