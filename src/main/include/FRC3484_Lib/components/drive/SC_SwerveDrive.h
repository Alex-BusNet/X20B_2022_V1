#ifndef SC_SWERVEDRIVE_H
#define SC_SWERVEDRIVE_H

#include <tuple>
#include "AHRS.h"
#include "frc/I2C.h"

#include "SC_SwerveModule.h"
#include "FRC3484_Lib/utils/SC_Datatypes.h"

namespace SC
{
	class SC_SwerveDrive
	{
	public:
		SC_SwerveDrive(SC::SC_SwerveConfig FL_Cfg, SC::SC_SwerveConfig FR_Cfg, SC::SC_SwerveConfig BL_Cfg, SC::SC_SwerveConfig BR_Cfg, frc::I2C::Port imu_port);
		~SC_SwerveDrive();

		void DriveToPosition(double X_SP, double Y_SP, double theta_SP);
		void Drive(double JS_X, double JS_Y, double JS_theta);


	private:
		AHRS *_imu; // NavX IMU
		SC::SC_SwerveModule *_FL, *_FR, *_BL, *_BR;

		

	};
}

#endif // SC_SWERVEDRIVE_H