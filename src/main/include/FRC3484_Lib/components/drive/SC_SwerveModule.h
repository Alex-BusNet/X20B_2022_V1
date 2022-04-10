#ifndef SC_SWERVEMODULE_H
#define SC_SWERVEMODULE_H

#include "ctre/phoenix/motorcontrol/can/WPI_TalonFX.h"
#include "ctre/phoenix/sensors/CANCoder.h"
#include "FRC3484_Lib/components/SC_PID.h"
#include "FRC3484_Lib/utils/SC_Datatypes.h"

#include "frc/geometry/Translation2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/kinematics/SwerveModuleState.h"

namespace SC
{
	class SC_SwerveModule
	{
	public:
		SC_SwerveModule(SC::SC_SwerveModuleConfig Cfg, units::velocity::meters_per_second_t MaxLinVelocity);
		~SC_SwerveModule();

		void SetInversionDirection(ctre::phoenix::motorcontrol::TalonFXInvertType DriveDir, ctre::phoenix::motorcontrol::TalonFXInvertType RotDir);
		void SetDriveScaleFactor(double SF);
		void SetRotScaleFactor(double SF);

		void Calculate(frc::SwerveModuleState SMS);

		/**
		 * @brief Rotates the wheel to 0° on the encoder
		 */
		void ReturnToZero();

		frc::Rotation2d GetSteerAngle();

	private:
		ctre::phoenix::motorcontrol::can::WPI_TalonFX *_motor_drive, *_motor_rot;
		ctre::phoenix::sensors::CANCoder *_encoder_rot;

		SC_PID *_pid_drive, *_pid_rot;

		frc::Translation2d _module_loc;

		double _maxDriveSpeed;

		double _driveSF, _rotSF;
		double _drivePV_eu, _rotPV_eu;		// EU = Engineering Units. Value represents physical behavior
		double _drivePV_pct, _rotPV_pct;	// Pct = Percent. This value is scaled to be a percentage of full scale (%FS).
		double _driveSP_eu, _rotSP_eu;
		double _driveSP_pct, _rotSP_pct;

		double _driveCV_pct, _driveCV_eu;	// CV_EU = Voltage
		double _rotCV_pct, _rotCV_eu;		// CV_EU = Voltage

		bool _s_wheelAligned;

	};
}

#endif // SC_SWERVEMODULE_H