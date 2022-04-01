#ifndef SC_MECANUMDRIVE_H
#define SC_MECANUMDRIVE_H

#include "units/angle.h"
#include "ctre/phoenix/motorcontrol/can/TalonSRX.h"
#include "FRC3484_Lib/utils/SC_Datatypes.h"
#include <frc/drive/Vector2d.h>
#include <frc/kinematics/MecanumDriveWheelSpeeds.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/MecanumDriveKinematics.h>
#include <frc/kinematics/MecanumDriveOdometry.h>
#include <cmath>

namespace SC
{
    class SC_MecanumDrive
    {
    public:
        SC_MecanumDrive();

        ~SC_MecanumDrive();

        /**
         * @brief   DriveCartesian
         */
        void DriveCartesian(double X, double Y, double zRotation, units::angle::degree_t gyro);

        /**
         * @brief   DrivePolar
         */
        void DrivePolar(double radius, units::angle::degree_t theta, double zRotation);

        void SetMaxWheelSpeed(units::meters_per_second_t mSpeed);

        frc::MecanumDriveWheelSpeeds GetWheelSpeedsSetpoint();
        
        frc::ChassisSpeeds GetChassisSpeed();

        double GetWheelOutput(SC::SC_Wheel wheelIdx);

    private:
        frc::ChassisSpeeds chassis;
        frc::MecanumDriveWheelSpeeds wheelSpeed_SP, wheelSpeed_PV;
        frc::MecanumDriveKinematics kinematics{ frc::Translation2d{-0.1_m,  0.1_m},
                                                frc::Translation2d{ 0.1_m,  0.1_m},
                                                frc::Translation2d{-0.1_m, -0.1_m},
                                                frc::Translation2d{ 0.1_m, -0.1_m}};
        
        frc::MecanumDriveOdometry odometry{ kinematics, 
                                            frc::Rotation2d{0_rad}, 
                                            frc::Pose2d{0_m, 0_m, 0_rad}};

        units::meters_per_second_t maxWheelSpeed = 5.0_mps;
    };
}

#endif