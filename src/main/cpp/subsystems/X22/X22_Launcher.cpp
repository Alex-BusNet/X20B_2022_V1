#include "subsystems/X22/X22_Launcher.h"

using namespace frc;
using namespace SC;
using namespace ctre::phoenix::motorcontrol::can;
using namespace ctre::phoenix::motorcontrol;

X22_Launcher::X22_Launcher(int CenterID, int OuterID, int TurretID, SC_Solenoid Loader, SC_Solenoid AngleCtrl, I2C::Port ColorSenPort)
{
	this->_launch_motor_center = new TalonFX(CenterID);
	this->_launch_motor_outer = new TalonFX(OuterID);

	this->_turret_motor = new VictorSPX(TurretID);

	this->_sol_loader = new Solenoid(Loader.CtrlID, Loader.CtrlType, Loader.Channel);
	this->_sol_angle = new Solenoid(AngleCtrl.CtrlID, AngleCtrl.CtrlType, AngleCtrl.Channel);

	this->_sen_loader = new SC_ColorSensor(ColorSenPort);

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

void X22_Launcher::Auto(bool Run)
{
	if(Run && (this->_outer_PID != NULL) && (this->_center_PID != NULL))
	{
		if(this->_center_PID->InManualMode()) { this->_center_PID->DisableManualMode(); }
		if(this->_outer_PID->InManualMode()) { this->_outer_PID->DisableManualMode(); }

		// TODO: Setup PID loop control
		if(this->_launch_motor_center != NULL) { this->_launch_motor_center->Set(ControlMode::PercentOutput, 0.0); }
		if(this->_launch_motor_outer != NULL) { this->_launch_motor_outer->Set(ControlMode::PercentOutput, 0.0); }

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

void X22_Launcher::SetCenterPIDTune(SC_PIDConstants PIDC)
{
	if(this->_center_PID != NULL) {this->_center_PID->SetPIDConstants(PIDC); }
}

void X22_Launcher::SetOuterPIDTune(SC_PIDConstants PIDC)
{
	if(this->_outer_PID != NULL) { this->_outer_PID->SetPIDConstants(PIDC); }
}

bool X22_Launcher::_HasCargoInLoader()
{
	if(this->_sen_loader != NULL) { return this->_sen_loader->GetDistance() < 1000.0; } // TODO: Get sensor distance when cargo is present.
	else { return false; }
}

void X22_Launcher::_SpoolFlywheel()
{

}