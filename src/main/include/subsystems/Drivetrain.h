#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "../FRC3484_Lib/subsystems/SC_MecanumDrive.h"
#include "frc/Solenoid.h"
#include "ctre/phoenix/motorcontrol/can/TalonSRX.h"

class Drivetrain : public SC::SC_MecanumDrive
{
public:
    Drivetrain();
    Drivetrain( std::tuple<int, int> chFR, 
                std::tuple<int, int> chFL, 
                std::tuple<int, int> chBR, 
                std::tuple<int, int> chBL,
                int ch_shift);

    ~Drivetrain();

    void Drive(double joystick_x, double joystick_y, double gyro, bool shift);

    void DriveAuto(double magnitude, double angle, double heading, bool shift);
    
    /**
     * @brief   Forces motors to drive with fixed values. Inputs are assumed
     *          to be percent output values between -1.0 and 1.0.
     * 
     *          All input values are internally limited before being assigned
     *          to the motor controllers.
     */
    void DriveDirect(double rawFR, double rawFL, double rawBR, double rawBL);

    /**
     * @brief   Ramp is used to control the slew rate (acceleration)
     *          of the drivetrain.
     */
    void Ramp();

    void SetDriveMode(SC::DriveMode dm);

    void StopMotors();

private:
    void _setOutputs();

    SC::SC_MecanumDrive *md;
    frc::Solenoid *shifter;

    SC::DriveMode defaultMode = SC::DriveMode::MECANUM;
    SC::DriveMode activeMode = SC::DriveMode::DEFAULT;

    // Declare the master and slave motor controllers.
    ctre::phoenix::motorcontrol::can::TalonSRX *FR, *FRs;
    ctre::phoenix::motorcontrol::can::TalonSRX *FL, *FLs;
    ctre::phoenix::motorcontrol::can::TalonSRX *BR, *BRs;
    ctre::phoenix::motorcontrol::can::TalonSRX *BL, *BLs;

};

#endif // DRIVETRAIN_H