#ifndef X22_DRIVETRAIN_H
#define X22_DRIVETRAIN_H


#include "ctre/phoenix/motorcontrol/can/TalonFX.h"

#include "frc/Solenoid.h"

#include "FRC3484_Lib/subsystems/SC_DifferentialDrive.h"
#include "FRC3484_Lib/utils/SC_Datatypes.h"

class X22_Drivetrain
{
public:
    X22_Drivetrain();
    X22_Drivetrain(units::length::inch_t trackWidth,
                   units::velocity::feet_per_second_t MaxVel, 
                   units::angular_velocity::degrees_per_second_t MaxRotVel,
                   std::tuple<int, int> LeftIDs, std::tuple<int, int> RightIDs, 
                   SC::SC_Solenoid shifter);

    ~X22_Drivetrain();

    void Drive(double Throttle, double Rotation);

private:
    SC::SC_DifferentialDrive *drive;
    ctre::phoenix::motorcontrol::can::TalonFX *Motor_Left_Master, *Motor_Left_Slave;
    ctre::phoenix::motorcontrol::can::TalonFX *Motor_Right_Master, *Motor_Right_Slave;

    double velocity;

    frc::Solenoid *_shifter;

    SC::SC_Range<double> _linVelRange;
    SC::SC_Range<double> _angVelRange;
};

#endif // X22_DRIVETRAIN_H