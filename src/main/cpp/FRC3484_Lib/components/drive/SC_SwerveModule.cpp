#include "FRC3484_Lib/components/drive/SC_SwerveModule.h"

using namespace SC;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

SC_SwerveModule::SC_SwerveModule(SC_SwerveConfig Cfg)
{
	_motor_drive = new WPI_TalonFX(Cfg.DriveID);
	_motor_rot = new WPI_TalonFX(Cfg.RotID);

	_pid_drive = new SC_PID(Cfg.DriveCoeff);
	_pid_rot = new SC_PID(Cfg.RotCoeff);

	_drivePV_eu = 0.0;
	_drivePV_pct = 0.0;

	_rotPV_eu = 0.0;
	_rotPV_pct = 0.0;
}

SC_SwerveModule::~SC_SwerveModule()
{
	if(_motor_drive != NULL) { delete _motor_drive; }
	if(_motor_rot != NULL) { delete _motor_rot; }
	if(_pid_drive != NULL) { delete _pid_drive; }
	if(_pid_rot != NULL) { delete _pid_rot; }
}

void SC_SwerveModule::SetInversionDirection(TalonFXInvertType DriveDir, TalonFXInvertType RotDir) { _motor_drive->SetInverted(DriveDir); _motor_rot->SetInverted(RotDir); }

void SC_SwerveModule::SetDriveScaleFactor(double SF) { this->_driveSF = SF; }

void SC_SwerveModule::SetRotScaleFactor(double SF) { this->_rotSF = SF; }

void SC_SwerveModule::Calculate(double DriveSP, double RotSP)
{
	// 
}