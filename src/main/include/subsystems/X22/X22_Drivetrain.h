#ifndef X22_DRIVETRAIN_H
#define X22_DRIVETRAIN_H

// #include "FRC3484_Lib/subsystems/SC_DifferentialDrive.h"

#include "Constants.h"
#include "Globals.h"

#include "FRC3484_Lib/utils/SC_Datatypes.h"
#include "FRC3484_Lib/utils/SC_Shuffleboard.h"

#include "units/length.h"
#include "units/velocity.h"
#include "units/angular_velocity.h"

#include "ctre/phoenix/motorcontrol/can/WPI_TalonFX.h"

#include "frc/drive/DifferentialDrive.h"
#include "frc/kinematics/DifferentialDriveWheelSpeeds.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/kinematics/DifferentialDriveKinematics.h"

// #include "frc/Solenoid.h"
#include "frc/DoubleSolenoid.h"

/*
#include "frc/shuffleboard/Shuffleboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableValue.h"
#include "wpi/StringMap.h"
*/

class X22_Drivetrain
{
public:
    X22_Drivetrain(units::length::inch_t trackWidth,
                   units::velocity::feet_per_second_t MaxVel, 
                   units::angular_velocity::degrees_per_second_t MaxRotVel,
                   std::tuple<int, int> LeftIDs, std::tuple<int, int> RightIDs, 
                   SC::SC_DoubleSolenoid shifter);

    ~X22_Drivetrain();

    void Drive(double Throttle, double Rotation, bool ShiftOverride);

    void Shift(bool ShiftLow, bool ShiftHigh);

private:

    double _CalcWheelVelocity(double counts);

    void _UpdateDashboard();
    void _InitDashboard();
    void _InitMotor(ctre::phoenix::motorcontrol::can::WPI_TalonFX* Motor, bool Invert, ctre::phoenix::motorcontrol::can::WPI_TalonFX* Master = NULL);

    //SC::SC_DifferentialDrive *drive;
    frc::DifferentialDrive              *drive;
    frc::DifferentialDriveKinematics    *ddKinematics;
    frc::DifferentialDrive::WheelSpeeds wsInput;
    frc::DifferentialDriveWheelSpeeds   ws_PV, ws_PVf;//, ws_chassis;
    frc::ChassisSpeeds                  cs_PV;

    ctre::phoenix::motorcontrol::can::WPI_TalonFX *Motor_Left_Master, *Motor_Left_Slave;
    ctre::phoenix::motorcontrol::can::WPI_TalonFX *Motor_Right_Master, *Motor_Right_Slave;

    double throttleDemand, rotationDemand, throttleCoeff;
    bool inHighGear, inLowGear;
    
    frc::DoubleSolenoid *_shifter;

    SC::SC_Range<double> _linVelRange;
    SC::SC_Range<double> _angVelRange;

    SC::SC_ABFilter<units::velocity::meters_per_second_t> *_ws_filter_left, *_ws_filter_right;

    // Dashboard outputs
    SC::SC_SBItem<double> *ntLeftOut, *ntRightOut;
    SC::SC_SBItem<double> *ntThrottleIn, *ntRotationIn;
    SC::SC_SBItem<bool> *ntInLowGear, *ntInHighGear;
    SC::SC_SBItem<double> *ntLeftVel, *ntRightVel;
    SC::SC_SBItem<double> *ntChassisVx, *ntChassisVy, *ntChassisOmega;

    // Dashboard inputs
    SC::SC_SBItem<double> *ntRampTime, *ntThrottleScale;

    /*
    nt::NetworkTableEntry ntLeftOut, ntRightOut;
    nt::NetworkTableEntry ntThrottleIn, ntRotationIn;
    nt::NetworkTableEntry ntInLowGear, ntInHighGear;
    nt::NetworkTableEntry ntRampTime;
    nt::NetworkTableEntry ntLeftVel, ntRightVel;
    nt::NetworkTableEntry ntChassisVx, ntChassisVy, ntChassisOmega;
    nt::NetworkTableEntry ntLowSF, ntHighSF;
    */

    wpi::StringMap<std::shared_ptr<nt::Value>> props_numbar =
    {
        std::make_pair("min", nt::Value::MakeDouble(-1.0)),
        std::make_pair("max", nt::Value::MakeDouble(1.0))
    };

    wpi::StringMap<std::shared_ptr<nt::Value>> props_diffdrive_lo =
    {
        std::make_pair("min", nt::Value::MakeDouble(-C_LO_GEAR_MAX_SPEED.value())),
        std::make_pair("max", nt::Value::MakeDouble(C_LO_GEAR_MAX_SPEED.value()))
        /*
        std::make_pair("Number of wheels", nt::Value::MakeDouble(6)),
        std::make_pair("Wheel Diameter", nt::Value::MakeDouble(60)), // Pixels
        std::make_pair("Show velocity vectors", nt::Value::MakeBoolean(true))
        */
    };

    wpi::StringMap<std::shared_ptr<nt::Value>> props_diffdrive_hi =
    {
        std::make_pair("min", nt::Value::MakeDouble(-C_HI_GEAR_MAX_SPEED.value())),
        std::make_pair("max", nt::Value::MakeDouble(C_HI_GEAR_MAX_SPEED.value()))
    };
};

#endif // X22_DRIVETRAIN_H