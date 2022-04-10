#include "FRC3484_Lib/components/drive/SC_SwerveModule.h"

#include "FRC3484_Lib/utils/SC_Constants.h"

using namespace SC;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;
using namespace ctre::phoenix::sensors;
using namespace frc;
using namespace units::velocity;

SC_SwerveModule::SC_SwerveModule(SC_SwerveModuleConfig Cfg, meters_per_second_t MaxLinVelocity)
{
	_motor_drive = new WPI_TalonFX(Cfg.DriveMotor.CANId);
	_motor_rot = new WPI_TalonFX(Cfg.RotMotor.CANId);
	_encoder_rot = new CANCoder(Cfg.EncID);

	_maxDriveSpeed = MaxLinVelocity.value();

	SetInversionDirection(Cfg.DriveMotor.InvDir, Cfg.RotMotor.InvDir);

	_pid_drive = new SC_PID(Cfg.DriveMotor.PIDCfg);
	_pid_rot = new SC_PID(Cfg.RotMotor.PIDCfg);

	_pid_drive->SetSPLimits(-100.0, 100.0);
	_pid_drive->SetPVLimits(-100.0, 100.0);
	_pid_drive->SetILimits(-100.0, 100.0);
	_pid_drive->SetDLimits(-100.0, 100.0);
	_pid_drive->SetCVLimits(-100, 100.0);

	_pid_rot->SetSPLimits(-100.0, 100.0);
	_pid_rot->SetPVLimits(-100.0, 100.0);
	_pid_rot->SetILimits(-100.0, 100.0);
	_pid_rot->SetDLimits(-100.0, 100.0);
	_pid_rot->SetCVLimits(-100.0, 100.0);

	_rotSF = 1.0; // CANCoder reports position in degrees.
	_driveSF = SC::C_TALONFX_RPM_SF;

	_drivePV_eu = 0.0;
	_drivePV_pct = 0.0;

	_rotPV_eu = 0.0;
	_rotPV_pct = 0.0;

	_module_loc = Cfg.ModuleLoc;

	_s_wheelAligned = false; // Clear wheel aligned status upon initialization
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

void SC_SwerveModule::Calculate(SwerveModuleState SMS)
{
	_rotPV_eu = _encoder_rot->GetPosition() * _rotSF;
	_drivePV_eu = _motor_drive->GetSelectedSensorVelocity() * _driveSF;

	// TODO: Set setpoints
	if(!_s_wheelAligned) { _driveSP_eu = 0; _rotSP_eu = 0; }
	else
	{
		_driveSP_pct = F_Scale(-_maxDriveSpeed, _maxDriveSpeed, -100.0, 100.0, SMS.speed.value());
		_rotSP_pct = F_Scale(-180.0, 180.0, -100.0, 100.0, SMS.angle.Degrees().value());
	}


	_driveCV_pct = _pid_drive->Calculate(_drivePV_pct, _driveSP_pct);
	_rotCV_pct = _pid_rot->Calculate(_rotPV_pct, _rotSP_pct);

	_motor_drive->Set(ControlMode::PercentOutput, _driveCV_pct / 100.0);
	_motor_rot->Set(ControlMode::PercentOutput, _rotCV_pct / 100.0 );

}

Rotation2d SC_SwerveModule::GetSteerAngle()
{
	return Rotation2d{units::make_unit<units::angle::degree_t>(_rotPV_eu)};
}

void SC_SwerveModule::ReturnToZero()
{

}