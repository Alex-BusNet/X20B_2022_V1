#include "subsystems/X22/X22_Launcher.h"
#include "Constants.h"

using namespace frc;
using namespace SC;
using namespace ctre::phoenix::motorcontrol::can;
using namespace ctre::phoenix::motorcontrol;

X22_Launcher::X22_Launcher(int CenterID, int OuterID, int TurretID, int TurretLSMin_Ch, int TurretLSMax_Ch, SC_Solenoid Loader, SC_Solenoid AngleCtrl)
{
	this->_launch_motor_center = new TalonFX(CenterID);
	this->_launch_motor_outer = new TalonFX(OuterID);

	this->_turret_motor = new VictorSPX(TurretID);
	this->_ls_turret_travel_min = ((TurretLSMin_Ch > -1) && (TurretLSMax_Ch != TurretLSMin_Ch)) ? new DigitalInput(TurretLSMin_Ch) : NULL;
	this->_ls_turret_travel_max = ((TurretLSMax_Ch > -1) && (TurretLSMax_Ch != TurretLSMin_Ch)) ? new DigitalInput(TurretLSMax_Ch) : NULL;
	this->_turretSearchDir = 1; // 1 = forward/CCW, -1 = reverse/CW

	this->_sol_loader = new Solenoid(Loader.CtrlID, Loader.CtrlType, Loader.Channel);
	this->_sol_angle = new Solenoid(AngleCtrl.CtrlID, AngleCtrl.CtrlType, AngleCtrl.Channel);

	this->_vision = new SC_Limelight(C_LL_ANGLE, C_LL_HEIGHT);
	this->_vision->SetTargetHeight(C_GOAL_HEIGHT);
	this->_vision->SetLEDMode(SC_LEDMode::LED_OFF);

	this->_center_PID = new SC_PID();
	this->_outer_PID = new SC_PID();

	this->_launch_motor_center->SetNeutralMode(NeutralMode::Coast);
	this->_launch_motor_center->SetInverted(false);
	
	this->_launch_motor_outer->SetNeutralMode(NeutralMode::Coast);
	this->_launch_motor_outer->SetInverted(true);
}

X22_Launcher::~X22_Launcher()
{
	if(this->_launch_motor_center != NULL) { delete this->_launch_motor_center; }
	if(this->_launch_motor_outer != NULL) { delete this->_launch_motor_outer; }
	if(this->_sol_loader != NULL) { delete this->_sol_loader; }
	if(this->_sol_angle != NULL) { delete this->_sol_angle; }
	if(this->_center_PID != NULL) { delete this->_center_PID; }
	if(this->_outer_PID != NULL) { delete this->_outer_PID; }
}

void X22_Launcher::Periodic(bool Run, double TurretVel, bool Launch, bool Eject)
{
	_TrackTarget(Run);
	_SpoolFlywheel(Run);

	_Manual(TurretVel, Launch, Eject);
}

void X22_Launcher::_Manual(double TurretVel, bool Launch, bool Eject)
{
	double __centerOut, __outerOut;
	bool __stopTurret;

	if((this->_launch_motor_center != NULL) && (this->_launch_motor_outer != NULL) && (this->_sol_loader != NULL) && (this->_turret_motor != NULL))
	{
		// Determine what launcher manual function should run
		if(Launch) { __centerOut = C_MANUAL_LAUNCH_VEL_CEN; __outerOut = C_MANUAL_LAUNCH_VEL_OUT; }
		else if(Eject) { __centerOut = C_EJECT_VEL_CEN; __outerOut = C_EJECT_VEL_OUT; }
		else { __centerOut = 0.0; __outerOut = 0.0; }

		// Check if the motors are at the target speed and the robot should shoot
		if(F_IsInRange_Offs(this->_launch_motor_center->GetMotorOutputPercent(), __centerOut, 0.05) && (Launch || Eject)) { this->_sol_loader->Set(true); }
		else { this->_sol_loader->Set(false); }
		
		// Stop the turret if the limit switch in the direction the turret is trying to move is pressed
		__stopTurret = (this->_ls_turret_travel_max->Get() && (TurretVel > 0.0)) || (this->_ls_turret_travel_min->Get() && (TurretVel < 0.0));

		// Set motor outputs
		this->_turret_motor->Set(ControlMode::PercentOutput, __stopTurret ? 0.0 : TurretVel);
		this->_launch_motor_center->Set(ControlMode::PercentOutput, __centerOut);
		this->_launch_motor_outer->Set(ControlMode::PercentOutput, __outerOut);
	}
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
	if(this->_ls_turret_travel_min != NULL) { return this->_ls_turret_travel_min->Get(); }
	else {return false; }
}

bool X22_Launcher::IsTurretAtMax()
{
	if(this->_ls_turret_travel_max != NULL) { return this->_ls_turret_travel_max->Get(); }
	else { return false; }
}

void X22_Launcher::_TrackTarget(bool Enable)
{
	if(Enable && (this->_vision != NULL) && (this->_turret_motor != NULL))
	{
		this->_vision->SetLEDMode(SC_LEDMode::AUTO);
		
		// Once the limelight finds a target, adjust the turret to keep it centered
		if(this->_vision->HasTarget())
		{
			// Turret PI Control
			this->_turretErr = 0 - this->_vision->GetOffsetX(); 
			this->_turret_P = this->_turretErr * C_TURRET_DEFAULT_KP;
			this->_turret_I = F_Limit(-100.0, 100.0, this->_turret_I + (this->_turretErr * C_TURRET_DEFAULT_KI * C_SCAN_TIME));

			this->_turret_CV = F_Scale(-100.0, 100.0, -1.0, -1.0, (this->_turret_P + this->_turret_I));

			this->_turret_motor->Set(ControlMode::PercentOutput, this->_turret_CV);

			this->_turretReady = F_IsInRange_Offs(this->_turretErr, 0.0, C_TURRET_READY_BAND);
		}
		else
		{	
			this->_turret_I = 0.0;

			// Search for target
			if(this->_ls_turret_travel_max->Get()) { this->_turretSearchDir = -1; } // Turret has reached max travel, move the other direction
			else if(this->_ls_turret_travel_min->Get()) { this->_turretSearchDir = 1; } // Turrent has reached min travel, move the other direction
			
			this->_turret_motor->Set(ControlMode::PercentOutput, C_TURRET_DEFAULT_SEEK_VEL * this->_turretSearchDir);
		}
	}
	else
	{
		if(this->_vision != NULL) { this->_vision->SetLEDMode(SC_LEDMode::LED_OFF); }

		if(this->_turret_motor != NULL) { this->_turret_motor->Set(ControlMode::PercentOutput, 0.0); }
	}
}

void X22_Launcher::_SpoolFlywheel(bool Enable)
{
	if((this->_center_PID != NULL) && (this->_outer_PID != NULL))
	{
		if(Enable)
		{
			if(this->_vision->HasTarget())
			{			
				// Turn off manual mode on the PID loops
				if(this->_center_PID->InManualMode()) { this->_center_PID->DisableManualMode(); }
				if(this->_outer_PID->InManualMode()) { this->_outer_PID->DisableManualMode(); }
				
				// Spool up the lauch wheels to the target distance
				this->_launch_dist = this->_vision->GetDistanceFromTarget();
				this->_center_PID->SetSP(F_XYCurve(C_LAUNCHER_CURVE_X, C_LAUNCHER_CEN_CURVE_Y, this->_launch_dist, 10));
				this->_center_PID->SetSP(F_XYCurve(C_LAUNCHER_CURVE_X, C_LAUNCHER_OUT_CURVE_Y, this->_launch_dist, 10));

				this->_sol_angle->Set(this->_launch_dist > C_LAUNCH_ANGLE_THRESHOLD);

				this->_center_wheel_ready = F_IsInRange_Offs(this->_launch_motor_center->GetSelectedSensorVelocity(), this->_center_PID->GetStatus().SP, C_LAUNCH_CEN_READY_BAND);
				this->_outer_wheel_ready = F_IsInRange_Offs(this->_launch_motor_outer->GetSelectedSensorVelocity(), this->_outer_PID->GetStatus().SP, C_LAUNCH_CEN_READY_BAND);

				this->_sol_loader->Set(this->_center_wheel_ready && this->_outer_wheel_ready && this->_turretReady);

			}
			else
			{
				if(!this->_center_PID->InManualMode()) { this->_center_PID->EnableManualMode(); }
				if(!this->_outer_PID->InManualMode()) { this->_outer_PID->EnableManualMode(); }
			}

			this->_launch_motor_center->Set(ControlMode::PercentOutput, this->_center_PID->Calculate(this->_launch_motor_center->GetSelectedSensorVelocity()));
			this->_launch_motor_outer->Set(ControlMode::PercentOutput, this->_outer_PID->Calculate(this->_launch_motor_outer->GetSelectedSensorVelocity()));

		}
		else
		{
			if((this->_center_PID != NULL) && (!this->_center_PID->InManualMode()))
			{ 
				this->_center_PID->EnableManualMode();
				this->_center_PID->SetCV(0.0); 
			}

			if((this->_outer_PID != NULL) && (!this->_center_PID->InManualMode()))
			{ 
				this->_outer_PID->EnableManualMode();
				this->_outer_PID->SetCV(0.0); 
			}
		}
	}
}