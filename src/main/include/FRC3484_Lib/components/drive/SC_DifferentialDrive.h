#ifndef SC_DIFFERENTIALDRIVE_H
#define SC_DIFFERENTIALDRIVE_H

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>

#include "FRC3484_Lib/components/SC_PID.h"

#include "units/length.h"
#include "units/velocity.h"
#include "units/angular_velocity.h"

namespace SC
{
    class SC_DifferentialDrive
    {
    public:
        SC_DifferentialDrive(units::inch_t trackWidth);
		SC_DifferentialDrive(units::inch_t trackWidth, 
							 units::velocity::feet_per_second_t MaxLinearVel,
							 units::angular_velocity::degrees_per_second_t MaxAngularVel);
        
		~SC_DifferentialDrive();

        void Drive_Vel(double VelSP,		// feet/sec
		 			   double zRotation,	// deg/sec
					   double VelPV,		// feet/sec
					   bool DirectDrive);

        void Drive_Pos();

		void SetPIDParams(SC_PIDConstants pidc);

		double GetWheelOutput(SC_Wheel side);

    private:
		frc::DifferentialDriveWheelSpeeds _DDIK(double tangVel, double zRot); // Diff. Drive Inverse Kinematics

		double maxLinearSpeed;
		double maxRotationSpeed;

        frc::ChassisSpeeds csInput, csPV;
        frc::DifferentialDriveWheelSpeeds wsInput, wsPV;

        frc::DifferentialDriveKinematics *ddKinematics;
        //frc::DifferentialDriveOdometry *ddOdometry;        

		// PID Loops
		SC_PID *pid;
			

    };
}

#endif // SC_DIFFERENTIALDRIVE_H