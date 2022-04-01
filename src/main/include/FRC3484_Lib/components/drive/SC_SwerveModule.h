#ifndef SC_SWERVEMODULE_H
#define SC_SWERVEMODULE_H

#include "ctre/phoenix/motorcontrol/can/WPI_TalonFX.h"
#include "FRC3484_Lib/components/SC_PID.h"
#include "FRC3484_Lib/utils/SC_Datatypes.h"

namespace SC
{
	class SC_SwerveModule
	{
	public:
		SC_SwerveModule(SC::SC_SwerveConfig Cfg);
		~SC_SwerveModule();

		void SetInversionDirection(ctre::phoenix::motorcontrol::TalonFXInvertType DriveDir, ctre::phoenix::motorcontrol::TalonFXInvertType RotDir);
		void SetDriveScaleFactor(double SF);
		void SetRotScaleFactor(double SF);

		void Calculate(double DriveSP, double RotSP);

	private:
		ctre::phoenix::motorcontrol::can::WPI_TalonFX *_motor_drive, *_motor_rot;

		SC_PID *_pid_drive, *_pid_rot;

		double _driveSF, _rotSF;
		double _drivePV_eu, _rotPV_eu;	// EU = Engineering Units. Value represents physical behavior
		double _drivePV_pct, _rotPV_pct;	// Pct = Percent. This value is scaled to be a percentage of full scale (%FS).

	};
}

#endif // SC_SWERVEMODULE_H