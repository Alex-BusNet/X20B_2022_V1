#ifndef SC_SWERVEDRIVE_H
#define SC_SWERVEDRIVE_H

#include <tuple>
#include "AHRS.h"
#include "frc/I2C.h"

#include "SC_SwerveModule.h"
#include "FRC3484_Lib/utils/SC_Datatypes.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "frc/kinematics/SwerveDriveOdometry.h"

#include "units/velocity.h"
#include "units/angular_velocity.h"
#include "units/length.h"
#include "units/angle.h"

namespace SC
{
	class SC_SwerveDrive
	{
	public:
		SC_SwerveDrive(SC::SC_SwerveConfig Cfg, units::velocity::meters_per_second_t MaxLinearSpeed, units::velocity::meters_per_second_t MaxAngularSpeed, frc::I2C::Port imu_port = frc::I2C::kOnboard);
		~SC_SwerveDrive();

		void DriveToPosition(double X_SP, double Y_SP, double theta_SP);
		void Drive(double JS_X, double JS_Y, double JS_theta);

		/**
		 * @brief Rotates all wheels so that they align to 0°
		 */
		void AlignWheels();


	private:
		AHRS *_imu; // NavX IMU
		SC::SC_SwerveModule *_FL, *_FR, *_BL, *_BR;

		double _maxLinSpeed, _maxAngSpeed;

		frc::SwerveDriveKinematics<4> *_sdKinematics;
		frc::SwerveDriveOdometry<4> *_sdOdometry;

	    frc::ChassisSpeeds _cs{0_mps, 0_mps, 0_rad_per_s};

	};
}

#endif // SC_SWERVEDRIVE_H