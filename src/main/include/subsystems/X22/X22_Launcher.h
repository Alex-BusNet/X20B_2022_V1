#ifndef X22_LAUNCHER_H
#define X22_LAUNCHER_H

#include "FRC3484_Lib/components/SC_Limelight.h"
#include "FRC3484_Lib/components/SC_ColorSensor.h"
#include "FRC3484_Lib/components/SC_PID.h"

#include "FRC3484_Lib/utils/SC_Datatypes.h"
#include "FRC3484_Lib/utils/SC_Functions.h"
#include "FRC3484_Lib/utils/SC_Shuffleboard.h"

#include "ctre/phoenix/motorcontrol/can/TalonFX.h"
#include "ctre/phoenix/motorcontrol/can/VictorSPX.h"

#include "frc/Solenoid.h"
#include "frc/DigitalInput.h"
#include "frc/filter/Debouncer.h"

#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableValue.h"


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
	void Periodic(bool Auto, double TurretVel, bool LongLaunch, bool ShortLaunch, bool Eject);


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
	void _TrackTarget();

	void _SpoolFlywheel();

	void _initDashboard();
	void _updateDashboard();
	void _updateNTEntries();

	bool _en_autoLaunch, _en_manLaunch, _en_manEject;
	bool _en_long, _en_short;

	bool _turret_ready, _center_wheel_ready, _outer_wheel_ready, _hasTarget, _use_alt_launch_angle, _discharge; // Internal statuses

	int _turretSearchDir;
	double _turretErr, _turret_P, _turret_I, _turret_CV;
	double _turret_man_vel; // Manual turret control velocity

	double _launch_dist;
	double _launch_cen_fps_sp, _launch_out_fps_sp, _launch_cen_fps_pv, _launch_out_fps_pv;
	double _launch_cen_sp, _launch_out_sp, _launch_cen_pv, _launch_out_pv;
	double _launch_cen_cv, _launch_out_cv, _launch_cen_fps_cv, _launch_out_fps_cv; // PID loop outputs

	double _turretKp, _turretKi, _launchCenKp, _launchCenKi, _launchOutKp, _launchOutKi;

	ctre::phoenix::motorcontrol::can::TalonFX *_launch_motor_center, *_launch_motor_outer;
	ctre::phoenix::motorcontrol::can::VictorSPX *_turret_motor; 

	frc::DigitalInput *_ls_turret_travel_min, *_ls_turret_travel_max; 

	frc::Solenoid *_sol_loader, *_sol_angle; 

	SC::SC_Limelight *_vision; 

	SC::SC_PID *_center_PID, *_outer_PID; 

	// Dashboard interaction
	nt::NetworkTableInstance _nt_inst;
	std::shared_ptr<nt::NetworkTable> _nt_table;

	// Debouncers
	frc::Debouncer *_dbnc_re_turret_min; // Turret min travel limit switch, debounce on rising edge (RE)
	frc::Debouncer *_dbnc_re_turret_max; // Turret max travel limit switch, debounce on rising edge (RE)
	frc::Debouncer *_dbnc_re_auto_launch; // Auto-launch function, debounce on rising edge (RE)
	frc::Debouncer *_dbnc_re_launch_ready; // Launch wheels and turret ready, debounce on rising edge (RE)
	frc::Debouncer *_dbnc_re_man_launch, *_dbnc_re_man_eject; // Manual launch control, debounce on rising edge (RE)
	frc::Debouncer *_dbnc_re_alt_angle; // Alternate launch angle selection; debounce on rising edge (RE)

	frc::Debouncer *_dly_re_manual_load;
	frc::Debouncer *_dly_re_discharge_hold;

	double C_LAUNCHER_CURVE_X[10] = 
	{
		0.0,
		4.0,
		8.0,
		12.0,
		16.0,
		20.0,
		24.0,
		28.0,
		0.0,
		0.0
	};

	double C_LAUNCHER_CEN_CURVE_Y[10] = 
	{
		0.0,
		0.30,
		0.35,
		0.40,
		0.45,
		0.50,
		0.55,
		0.60,
		0.0,
		0.0
	};

	double C_LAUNCHER_OUT_CURVE_Y[10] = 
	{
		0.0,
		0.20,
		0.25,
		0.30,
		0.45,
		0.40,
		0.45,
		0.50,
		0.0,
		0.0
	};

};


#endif // X22_LAUNCHER_H
