#include "subsystems/X22/X22_Launcher.h"
#include "subsystems/X22/X22_Constants.h"
#include "Globals.h"

#include <frc/DriverStation.h>

using namespace frc;
using namespace SC;
using namespace nt;
using namespace ctre::phoenix::motorcontrol::can;
using namespace ctre::phoenix::motorcontrol;

X22_Launcher::X22_Launcher(int CenterID, int OuterID, int TurretID, int TurretLSMin_Ch, int TurretLSMax_Ch, SC_Solenoid Loader, SC_Solenoid AngleCtrl)
{
	_launch_motor_center = new TalonFX(CenterID);
	_launch_motor_outer = new TalonFX(OuterID);

	_turret_motor = new VictorSPX(TurretID);
	_ls_turret_travel_min = ((TurretLSMin_Ch > -1) && (TurretLSMax_Ch != TurretLSMin_Ch)) ? new DigitalInput(TurretLSMin_Ch) : NULL;
	_ls_turret_travel_max = ((TurretLSMax_Ch > -1) && (TurretLSMax_Ch != TurretLSMin_Ch)) ? new DigitalInput(TurretLSMax_Ch) : NULL;
	_turretSearchDir = 1; // 1 = forward/CCW, -1 = reverse/CW

	_sol_loader = new Solenoid(Loader.CtrlID, Loader.CtrlType, Loader.Channel);
	_sol_angle = new Solenoid(AngleCtrl.CtrlID, AngleCtrl.CtrlType, AngleCtrl.Channel);

	_vision = new SC_Limelight(C_LL_HEIGHT, C_LL_ANGLE);
	_vision->SetTargetHeight(C_GOAL_HEIGHT);
	_vision->SetLEDMode(SC_LEDMode::LED_OFF);
	_vision->SetDriverCam();

	_center_PID = new SC_PID(SC_PIDConstants{C_LAUNCH_CEN_DEFAULT_KP, C_LAUNCH_CEN_DEFAULT_KI, 0.0, 25.0});
	_outer_PID = new SC_PID(SC_PIDConstants{C_LAUNCH_OUT_DEFAULT_KP, C_LAUNCH_OUT_DEFAULT_KI, 0.0, 25.0});

	_center_PID->Setdt(C_SCAN_TIME);
	_center_PID->SetCV(0.0);
	_center_PID->SetManRate(2); // 0.1% increase per cycle (5% per second) when the CV is changed in manual mode
	_center_PID->Enable();
	_center_PID->EnableManualMode();
	// _center_PID->Disable();

	_outer_PID->Setdt(C_SCAN_TIME);
	_outer_PID->SetCV(0.0);
	_outer_PID->SetManRate(2); // 0.1% CV increase per cycle (5% per second) when the CV is changed in manual mode
	_outer_PID->Enable();
	_outer_PID->EnableManualMode();

	// _outer_PID->Disable();

	_launch_motor_center->SetNeutralMode(NeutralMode::Coast);
	_launch_motor_center->SetInverted(false);
	_launch_cen_sp = _launch_cen_fps_sp = 0.0;
	_launch_cen_pv = _launch_cen_fps_pv = 0.0;
	_launch_cen_cv = 0.0;
	_center_wheel_ready = false;
	
	_launch_motor_outer->SetNeutralMode(NeutralMode::Coast);
	_launch_motor_outer->SetInverted(true);
	_launch_out_sp = _launch_out_fps_sp = 0.0;
	_launch_out_pv = _launch_out_fps_pv = 0.0;
	_launch_out_cv = 0.0;
	_outer_wheel_ready = false;

	_launch_dist = 0.0;
	_use_alt_launch_angle = false;
	_en_autoLaunch = false;
	_en_manEject = false;
	_en_manLaunch = false;
	_en_short = false;
	_en_long = false;

	_dbnc_re_turret_max = new Debouncer(C_TURRET_MAX_DBNC_TIME, Debouncer::kRising);
	_dbnc_re_turret_min = new Debouncer(C_TURRET_MIN_DBNC_TIME, Debouncer::kRising);

	_dbnc_re_auto_launch = new Debouncer(C_AUTOLAUNCH_DBNC_TIME, Debouncer::kRising);
	_dbnc_re_launch_ready = new Debouncer(C_CEN_READY_DBNC_TIME, Debouncer::kRising);
	
	_dbnc_re_man_eject = new Debouncer(C_MANUEJECT_DBNC_TIME, Debouncer::kRising);
	_dbnc_re_man_launch = new Debouncer(C_MANULAUNCH_DBNC_TIME, Debouncer::kRising);
	_dbnc_re_alt_angle = new Debouncer(C_ALT_ANGLE_DBNC_TIME, Debouncer::kRising);

	_dly_re_manual_load = new Debouncer(2.0_s, Debouncer::kRising);
	_dly_re_discharge_hold = new Debouncer(1.0_s, Debouncer::kRising);

	_initDashboard();
}

X22_Launcher::~X22_Launcher()
{ 
	if(_launch_motor_center != NULL) {delete _launch_motor_center; }
	if(_launch_motor_outer != NULL) {delete _launch_motor_outer; }
	if(_turret_motor != NULL) { delete _turret_motor; }

	if(_ls_turret_travel_min != NULL) {delete _ls_turret_travel_min; }
	if(_ls_turret_travel_max != NULL) {delete _ls_turret_travel_max; }
	
	if(_sol_loader != NULL) {delete _sol_loader; }
	if(_sol_angle != NULL) {delete _sol_angle; }

	if(_center_PID != NULL) {delete _center_PID; }
	if(_outer_PID != NULL) {delete _outer_PID; }

	if(_vision != NULL) { delete _vision; }

	if(_dbnc_re_alt_angle != NULL) { delete _dbnc_re_alt_angle; }
	if(_dbnc_re_auto_launch != NULL) { delete _dbnc_re_auto_launch; }
	if(_dbnc_re_man_eject != NULL) { delete _dbnc_re_man_eject; }
	if(_dbnc_re_man_launch != NULL) { delete _dbnc_re_man_launch; }
	if(_dbnc_re_turret_max != NULL) { delete _dbnc_re_turret_max; }
	if(_dbnc_re_turret_min != NULL) { delete _dbnc_re_turret_min; }
	if(_dbnc_re_launch_ready != NULL) { delete _dbnc_re_launch_ready; }
	if(_dly_re_manual_load != NULL) { delete _dly_re_manual_load; }

}

void X22_Launcher::Periodic(bool Run, double TurretVel, bool LongLaunch, bool ShortLaunch, bool Eject)
{
	//
	// _updateNTEntries();
	_vision->SetDriverCam();
	_vision->SetLEDMode(SC_LEDMode::LED_OFF);
	
	// Update common statuses
	_hasTarget = _vision->HasTarget();
	_launch_cen_fps_pv = _launch_motor_center->GetSelectedSensorVelocity(0) * C_LAUNCH_SCALE_FACTOR;
	_launch_out_fps_pv = _launch_motor_outer->GetSelectedSensorVelocity(0) * C_LAUNCH_SCALE_FACTOR;

	_launch_cen_pv = F_Scale(0.0, C_LAUNCH_MAX_WHEEL_VEL, 0.0, 100.0, _launch_cen_fps_pv);
	_launch_out_pv = F_Scale(0.0, C_LAUNCH_MAX_WHEEL_VEL, 0.0, 100.0, _launch_out_fps_pv);

	_turret_man_vel = F_Scale(-1.0, 1.0, -C_TURRET_MAX_DEMAND, C_TURRET_MAX_DEMAND, F_Deadband(TurretVel, C_DRIVE_DEADBAND));

	if(_use_alt_launch_angle)
	{
		_vision->SetCameraAngle(C_LL_ANGLE_ALT);
		_vision->SetLensHeight(C_LL_HEIGHT_ALT);
	}
	else
	{
		_vision->SetCameraAngle(C_LL_ANGLE);
		_vision->SetLensHeight(C_LL_HEIGHT);
	}

	// Debounce input signals
	if(_dbnc_re_auto_launch != NULL) { _en_autoLaunch =_dbnc_re_auto_launch->Calculate(Run); }
	else { _en_autoLaunch = Run; }

	if(_dbnc_re_man_launch != NULL) { _en_manLaunch = _dbnc_re_man_launch->Calculate(LongLaunch || ShortLaunch); }
	else { _en_manLaunch = LongLaunch; }

	_en_long = _en_manLaunch && LongLaunch;
	_en_short = _en_manLaunch && ShortLaunch; 

	if(_dbnc_re_man_eject != NULL) { _en_manEject = _dbnc_re_man_eject->Calculate(Eject); }
	else { _en_manEject = Eject; }

	// Handle turret and launcher operations
	// _TrackTarget();
	_SpoolFlywheel();

	// Alternate launch angle activation debounce
	if(_dbnc_re_alt_angle != NULL) { _use_alt_launch_angle = _dbnc_re_alt_angle->Calculate(_launch_dist > C_LAUNCH_ANGLE_THRESHOLD) || (_en_long); }
	else { _use_alt_launch_angle = (_launch_dist > C_LAUNCH_ANGLE_THRESHOLD) || (_en_long); }

	// Loader Solenoid activation debounce
	if(_dbnc_re_launch_ready != NULL) 
	{ 
		_discharge = (_dbnc_re_launch_ready->Calculate((_center_wheel_ready && _outer_wheel_ready) || _en_manEject) && !_discharge)
						|| (_discharge && (_en_manLaunch || _en_manEject)); 
	}
	else { _discharge = ( _center_wheel_ready && _outer_wheel_ready) || (_en_manLaunch || _en_manEject); }

	// Set outputs
	_sol_angle->Set(_use_alt_launch_angle);
	_sol_loader->Set(_discharge);

	// _launch_motor_center->Set(ControlMode::PercentOutput, F_Scale(0.0, 100.0, 0.0, C_LAUNCH_MAX_DEMAND_PCT, _launch_cen_cv));
	// _launch_motor_outer->Set(ControlMode::PercentOutput, F_Scale(0.0, 100.0, 0.0, C_LAUNCH_MAX_DEMAND_PCT, _launch_out_cv));

	_launch_motor_center->Set(ControlMode::PercentOutput, F_Scale(0.0, 100.0, 0.0, 1.0, _launch_cen_cv));
	_launch_motor_outer->Set(ControlMode::PercentOutput, F_Scale(0.0, 100.0, 0.0, 1.0, _launch_out_cv));

	// Post launcher statuses to network tables
	_updateDashboard();
}

void X22_Launcher::SetCenterPIDTune(SC_PIDConstants PIDC)
{
	if(this->_center_PID != NULL) {this->_center_PID->SetPIDConstants(PIDC); }
}

void X22_Launcher::SetOuterPIDTune(SC_PIDConstants PIDC)
{
	if(this->_outer_PID != NULL) { this->_outer_PID->SetPIDConstants(PIDC); }
}

bool X22_Launcher::IsTurretAtMin()
{
	if((_ls_turret_travel_min != NULL) && (_dbnc_re_turret_min != NULL)) 
	{ 
		return _dbnc_re_turret_min->Calculate(!_ls_turret_travel_min->Get()); // Limit switch is wired NC
	}
	else { return false; }
}

bool X22_Launcher::IsTurretAtMax()
{
	if((_ls_turret_travel_max != NULL) && (_dbnc_re_turret_max != NULL)) 
	{ 
		return _dbnc_re_turret_max->Calculate(!_ls_turret_travel_max->Get()); // Limit switch is wired NC
	}
	else { return false; }
}

/*===================*/
/* Private Functions */
/*===================*/

void X22_Launcher::_TrackTarget()
{
	bool __stopTurret;
	if(_en_autoLaunch && (_vision != NULL) && (_turret_motor != NULL))
	{
		_vision->SetLEDMode(SC_LEDMode::AUTO);
		_vision->SetPipeline(0);

		//once the limelight finds a target, adjust the turret to keep it centered
		if(_hasTarget)
		{
			// TURRET PID Control
			_turretErr = 0 + _vision->GetOffsetX();
			_turret_P = _turretErr * _turretKp;
			_turret_I = F_Limit(-100.0, 100.0, (_turret_I + (_turretErr * _turretKi * C_SCAN_TIME_SEC)));

			_turret_CV = F_Scale(-100.0, 100.0, -C_TURRET_MAX_DEMAND, C_TURRET_MAX_DEMAND, (_turret_P + _turret_I));

			_turret_motor->Set(ControlMode::PercentOutput, _turret_CV);

			_turret_ready = F_IsInRange_Offs(_turretErr, 0.0, C_TURRET_READY_BAND);
		}
		else
		{
			_turret_I = 0.0;

			//Search For Target
			if(IsTurretAtMax()) { _turretSearchDir = -1; } //turret has reached max travel, move the other direction 
			else if (IsTurretAtMin()) { _turretSearchDir = 1; } //turret has reached min travel, move the other direction

			_turret_motor->Set(ControlMode::PercentOutput, C_TURRET_DEFAULT_SEEK_VEL * _turretSearchDir);
			_turret_ready = false;
		}
	}
	else
	{
		if(_vision != NULL) 
		{
			_vision->SetLEDMode(SC_LEDMode::LED_OFF);
			_vision->SetDriverCam(); 
		}

		if(_turret_motor != NULL) 
		{
			//Stop the turret if the limit switch in the direction the turet tis trying to move is pressed
			__stopTurret = ((this->IsTurretAtMax() && (_turret_man_vel < 0.0)) || (this->IsTurretAtMin() && (_turret_man_vel > 0.0)));
			_turret_ready = _en_manLaunch || _en_manEject;
			//Set motor outputs
			this->_turret_motor->Set(ControlMode::PercentOutput, __stopTurret ? 0.0 : _turret_man_vel);
		}
	}
}

void X22_Launcher::_SpoolFlywheel()
{
	if((_center_PID != NULL) && (_outer_PID != NULL))
	{
		//turn off manual mode on the PID loops
		if (_en_autoLaunch || _en_manLaunch) 
		{ 
			_center_PID->DisableManualMode(); 
			_outer_PID->DisableManualMode(); 
		}
		else 
		{
			if(!_center_PID->InManualMode()) 
			{
				_center_PID->EnableManualMode();
				// _center_PID->SetCV(0.0);
				// _center_PID->Reset();
			}

			if(!_outer_PID->InManualMode())
			{
				_outer_PID->EnableManualMode();
				// _outer_PID->SetCV(0.0);
				// _outer_PID->Reset();
			}	 
		}

		if(_en_autoLaunch && _hasTarget)
		{
			//spool up the launch wheels to the target destination
			_launch_dist = _vision->GetDistanceFromTarget();

			// Set the launch speed, bounded at the system allowed maximum
			_launch_cen_fps_sp = F_Limit(0.0, C_LAUNCH_MAX_WHEEL_VEL, F_XYCurve(C_LAUNCHER_CURVE_X, C_LAUNCHER_CEN_CURVE_Y, _launch_dist, 8));
			_launch_out_fps_sp = F_Limit(0.0, C_LAUNCH_MAX_WHEEL_VEL, F_XYCurve(C_LAUNCHER_CURVE_X, C_LAUNCHER_OUT_CURVE_Y, _launch_dist, 8));
		}
		else 
		{
			if(_en_manLaunch)
			{
				if(_en_long)
				{
					_launch_cen_fps_sp = C_LONG_LAUNCH_VEL_CEN;
					_launch_out_fps_sp = C_LONG_LAUNCH_VEL_OUT;
				}
				else if (_en_short)
				{
					_launch_cen_fps_sp = C_MANUAL_LAUNCH_VEL_CEN;
					_launch_out_fps_sp = C_MANUAL_LAUNCH_VEL_OUT;
				}
			}
			else if(_en_manEject || (frc::DriverStation::IsTeleopEnabled() && frc::DriverStation::IsFMSAttached()))
			{
				_launch_cen_fps_sp = C_EJECT_VEL_CEN;
				_launch_out_fps_sp = C_EJECT_VEL_OUT;
			}
			else
			{
				_launch_cen_fps_sp = C_EJECT_VEL_CEN;
				_launch_out_fps_sp = C_EJECT_VEL_OUT;
			}
		}

		// Scale the feet/second speed to a percentage of full scale
		_launch_cen_sp = F_Scale(0.0, C_LAUNCH_MAX_WHEEL_VEL, 0.0, 100.0, _launch_cen_fps_sp);
		_launch_out_sp = F_Scale(0.0, C_LAUNCH_MAX_WHEEL_VEL, 0.0, 100.0, _launch_out_fps_sp);

		// For launching, the SP is changed. For ejection and wind-down, the CV is changed.
		if(_en_autoLaunch || _en_manLaunch) 
		{ 
			_center_PID->SetSP(_launch_cen_sp); 
			_outer_PID->SetSP(_launch_out_sp); 
		}
		else
		{ 
			_center_PID->SetCV(_launch_cen_sp); 
			_outer_PID->SetCV(_launch_out_sp); 
		}

		_launch_cen_cv = _center_PID->Calculate(_launch_cen_pv);
		_launch_out_cv = _outer_PID->Calculate(_launch_out_pv);

		_launch_cen_fps_cv = F_Scale(0.0, 100.0, 0.0, C_LAUNCH_MAX_WHEEL_VEL, _launch_cen_cv);
		_launch_out_fps_cv = F_Scale(0.0, 100.0, 0.0, C_LAUNCH_MAX_WHEEL_VEL, _launch_out_cv);

	}
	/*
	else // PID Loops don't exist; run with manual launch vel only.
	{				
		if(_en_manLaunch)
		{
			_launch_cen_fps_sp = C_MANUAL_LAUNCH_VEL_CEN;
			_launch_out_fps_sp = C_MANUAL_LAUNCH_VEL_OUT;
		}
		else if(_en_manEject)
		{
			_launch_cen_fps_sp = C_EJECT_VEL_CEN;
			_launch_out_fps_sp = C_EJECT_VEL_OUT;
		}
		else
		{
			_launch_cen_fps_sp = 0.0;
			_launch_out_fps_sp = 0.0;	
		}		

		_launch_dist = 0.0;

		// Scale the feet/second speed to a percentage of full scale
		_launch_cen_cv = _launch_cen_sp = F_Scale(0.0, C_LAUNCH_MAX_WHEEL_VEL, 0.0, C_LAUNCH_MAX_DEMAND_PCT, _launch_cen_fps_sp);
		_launch_out_cv = _launch_out_sp = F_Scale(0.0, C_LAUNCH_MAX_WHEEL_VEL, 0.0, C_LAUNCH_MAX_DEMAND_PCT, _launch_out_fps_sp);
	}
	*/
	
	// Evaluate launcher ready statuses
	_center_wheel_ready = (F_IsInRange_Offs(_launch_cen_pv, _launch_cen_sp, C_LAUNCH_CEN_READY_BAND) 
							&& ((_en_autoLaunch && _hasTarget) || (_en_manLaunch || _en_manEject))); // Auto mode
						  //|| ((_en_manLaunch || _en_manEject) && F_IsInRange_Offs(_launch_cen_pv, _launch_cen_sp, C_LAUNCH_CEN_READY_BAND)); 	// Manual mode

	_outer_wheel_ready = (F_IsInRange_Offs(_launch_out_pv, _launch_out_sp, C_LAUNCH_OUT_READY_BAND) 
							&& ((_en_autoLaunch && _hasTarget) || (_en_manLaunch || _en_manEject))); // Auto mode
						  //|| ((_en_manLaunch || _en_manEject) && F_IsInRange_Offs(_launch_out_pv, _launch_out_sp, C_LAUNCH_OUT_READY_BAND));	// Manual mode
}

void X22_Launcher::_initDashboard()
{
	this->_nt_inst = NetworkTableInstance::GetDefault();
	this->_nt_table = this->_nt_inst.GetTable("X22");

	//display fields
	this->_nt_table->PutBoolean("Launcher Inner Ready", false);
	this->_nt_table->PutBoolean("Launcher Outer Ready", false);
	this->_nt_table->PutBoolean("Turret Ready", false);
	this->_nt_table->PutBoolean("Turret Min LS", false);
	this->_nt_table->PutBoolean("Turret Max LS", false);
	this->_nt_table->PutBoolean("Has Target", false);

	_nt_table->PutBoolean("Auto Launch", false);
	_nt_table->PutBoolean("Manual Launch", false);
	_nt_table->PutBoolean("Manual Eject", false);

	this->_nt_table->PutNumber("Turret Err", false);
	this->_nt_table->PutNumber("LC Vel SP", 0.0);
	this->_nt_table->PutNumber("LC Vel PV", 0.0);
	this->_nt_table->PutNumber("LO Vel SP", 0.0);
	this->_nt_table->PutNumber("LO Vel PV", 0.0);
	_nt_table->PutNumber("LC CV", 0.0);
	_nt_table->PutNumber("LO CV", 0.0);
	_nt_table->PutNumber("Launch Dist", 0.0);
	_nt_table->PutNumber("LC Error", 0.0);
	_nt_table->PutNumber("LO Error", 0.0);
	_nt_table->PutBoolean("LC ManMode", false);
	_nt_table->PutBoolean("LO ManMode", false);

	this->_nt_table->PutNumber("Turret Kp - Set", 0.0);
	this->_nt_table->PutNumber("Turret Ki - Set", 0.0);
	this->_nt_table->PutNumber("LC Kp - Set", 0.0);
	this->_nt_table->PutNumber("LC Ki - Set", 0.0);
	this->_nt_table->PutNumber("LO Kp - Set", 0.0);
	this->_nt_table->PutNumber("LO Ki - Set", 0.0);

	//input fields
	this->_nt_table->PutNumber("Turret Kp", C_TURRET_DEFAULT_KP);
	this->_nt_table->PutNumber("Turret Ki", C_TURRET_DEFAULT_KI);
	this->_nt_table->PutNumber("LC_Kp", C_LAUNCH_CEN_DEFAULT_KP);
	this->_nt_table->PutNumber("LC_Ki", C_LAUNCH_CEN_DEFAULT_KI);
	this->_nt_table->PutNumber("LO_Kp", C_LAUNCH_OUT_DEFAULT_KP);
	this->_nt_table->PutNumber("LO_Ki", C_LAUNCH_OUT_DEFAULT_KI);
}

void X22_Launcher::_updateDashboard()
{
	this->_nt_table->PutBoolean("Launcher Inner Ready", _center_wheel_ready);
	this->_nt_table->PutBoolean("Launcher Outer Ready", _outer_wheel_ready);
	this->_nt_table->PutBoolean("Turret Ready", _turret_ready);
	this->_nt_table->PutBoolean("Turret Min LS", IsTurretAtMin());
	this->_nt_table->PutBoolean("Turret Max LS", IsTurretAtMax());
	this->_nt_table->PutBoolean("Has Target", _hasTarget);

	_nt_table->PutBoolean("Auto Launch", _en_autoLaunch);
	_nt_table->PutBoolean("Manual Launch", _en_manLaunch);
	_nt_table->PutBoolean("Manual Eject", _en_manEject);

	_nt_table->PutNumber("Turret Err", _turretErr);
	_nt_table->PutNumber("LC Vel SP", _launch_cen_fps_sp);
	_nt_table->PutNumber("LC Vel PV", _launch_cen_fps_pv);
	_nt_table->PutNumber("LO Vel SP", _launch_out_fps_sp);
	_nt_table->PutNumber("LO Vel PV", _launch_out_fps_pv);
	_nt_table->PutNumber("LC CV", _launch_cen_fps_cv);
	_nt_table->PutNumber("LO CV", _launch_out_fps_cv);

	_nt_table->PutNumber("Launch Dist", _launch_dist);
	_nt_table->PutNumber("LC Error", _center_PID->GetError());
	_nt_table->PutNumber("LO Error", _outer_PID->GetError());
	_nt_table->PutBoolean("LC ManMode", _center_PID->InManualMode());
	_nt_table->PutBoolean("LO ManMode", _outer_PID->InManualMode());

	this->_nt_table->PutNumber("Turret Kp - Set", _turretKp);
	this->_nt_table->PutNumber("Turret Ki - Set", _turretKi);
	this->_nt_table->PutNumber("LC_Kp_Set", _launchCenKp);
	this->_nt_table->PutNumber("LC_Ki_Set", _launchCenKi);
	this->_nt_table->PutNumber("LO_Kp_Set", _launchOutKp);
	this->_nt_table->PutNumber("LO_Ki_Set", _launchOutKi);

}

void X22_Launcher::_updateNTEntries()
{
	this->_turretKp = this->_nt_table->GetNumber("Turret Kp", C_TURRET_DEFAULT_KP);
	this->_turretKi = this->_nt_table->GetNumber("Turret Ki", C_TURRET_DEFAULT_KI);
	this->_launchCenKp = this->_nt_table->GetNumber("LC_Kp", C_LAUNCH_CEN_DEFAULT_KP);
	this->_launchCenKi = this->_nt_table->GetNumber("LC_Ki", C_LAUNCH_CEN_DEFAULT_KI);
	this->_launchOutKp = this->_nt_table->GetNumber("LO_Kp", C_LAUNCH_CEN_DEFAULT_KP);
	this->_launchOutKi = this->_nt_table->GetNumber("LO_Ki", C_LAUNCH_OUT_DEFAULT_KI);

	if(this->_outer_PID != NULL) 
	{
		this->_outer_PID->SetKp(this->_launchOutKp);
		this->_outer_PID->SetKi(this->_launchOutKi);
	}
	
	if(this->_center_PID != NULL)
 	{
		this->_center_PID->SetKp(this->_launchCenKp);
		this->_center_PID->SetKi(this->_launchCenKi);
	}
}