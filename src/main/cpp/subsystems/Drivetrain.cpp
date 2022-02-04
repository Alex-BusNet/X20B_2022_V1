#include "subsystems/Drivetrain.h"
#include "units/angle.h"
#include "FRC3484_Lib/utils/SC_Functions.h"
#include "Constants.h"

using namespace SC;
using namespace frc;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;


Drivetrain::Drivetrain()
{
    md = nullptr;
    shifter = nullptr;

    // Initialize the Master controllers
    FR = nullptr;
    FL = nullptr;
    BR = nullptr;
    BL = nullptr;

    // Initalize the slave controllers
    FRs = nullptr;
    FLs = nullptr;
    BRs = nullptr;
    BLs = nullptr;
}

Drivetrain::Drivetrain( std::tuple<int, int> chFR, 
                        std::tuple<int, int> chFL, 
                        std::tuple<int, int> chBR, 
                        std::tuple<int, int> chBL, 
                        int ch_shift)
{
    md = new SC_MecanumDrive();
    shifter = new Solenoid(0, frc::PneumaticsModuleType::CTREPCM, ch_shift);

    int sCh = -1;

    // Initialize front right wheel and slave controller
    if(chFR != C_BLANK_IDS) 
    { 
        FR = new TalonSRX(std::get<0>(chFR));
        sCh = std::get<1>(chFR);

        if(sCh > -1)
        {
            FRs = new TalonSRX(sCh);
            FRs->Follow(*FR);
        }
        else
        {
            FRs = nullptr;
        }
        
    } 
    else
    {
        FR = nullptr;
        FRs = nullptr;
    }
    

    // Initialize front left wheel and slave controller
    if(chFL != C_BLANK_IDS) 
    { 
        FR = new TalonSRX(std::get<0>(chFL));
        
        sCh = std::get<1>(chFL);
        if(sCh > -1)
        {
            FLs = new TalonSRX(sCh);
            FLs->Follow(*FL);
        }
        else
        {
            FLs = nullptr;
        }
        
    } 
    else
    {
        FL = nullptr;
        FLs = nullptr;
    }

    // Initialize back right wheel and slave controller
    if(chBR != C_BLANK_IDS) 
    { 
        BR = new TalonSRX(std::get<0>(chBR));

        sCh = std::get<1>(chBR);
        if(sCh > -1)
        {
            BRs = new TalonSRX(sCh);
            BRs->Follow(*BR);
        }
        else
        {
            BRs = nullptr;
        }  
    } 
    else
    {
        BR = nullptr;
        BRs = nullptr;
    }

    // Initialize back left wheel and slave controller
    if(chBL != C_BLANK_IDS) 
    { 
        BL = new TalonSRX(std::get<0>(chBL));

        sCh = std::get<1>(chBL);
        if(sCh > -1)
        {
            BLs = new TalonSRX(sCh);
            BLs->Follow(*BL);
        }
        else
        {
            BLs = nullptr;
        }
    } 
    else
    {
        BL = nullptr;
        BLs = nullptr;
    }

    if(FR != nullptr) { FR->SetNeutralMode(NeutralMode::Coast); }
    if(FL != nullptr) { FL->SetNeutralMode(NeutralMode::Coast); }
    if(BR != nullptr) { BR->SetNeutralMode(NeutralMode::Coast); }
    if(BL != nullptr) { BL->SetNeutralMode(NeutralMode::Coast); }
}

Drivetrain::~Drivetrain()
{
    if(md != nullptr) { delete md; md = nullptr; }
    if(shifter != nullptr) { delete shifter; shifter = nullptr; }

    if(FRs != nullptr) { delete FRs; FRs = nullptr; }
    if(FLs != nullptr) { delete FLs; FLs = nullptr; }
    if(BRs != nullptr) { delete BRs; BRs = nullptr; }
    if(BLs != nullptr) { delete BLs; BLs = nullptr; }

    if(FR != nullptr) { delete FR; FR = nullptr; }
    if(FL != nullptr) { delete FL; FL = nullptr; }
    if(BR != nullptr) { delete BR; BR = nullptr; }
    if(BL != nullptr) { delete BL; BL = nullptr; }

}

void Drivetrain::Drive(double joystick_x, double joystick_y, double gyro, bool shift)
{
    if(shifter != nullptr)
        shifter->Set(shift);

    if(md != nullptr)
    {
        md->SetMaxWheelSpeed(shift ? C_LOW_GEAR_MAX_SPEED : C_HIGH_GEAR_MAX_SPEED);

        md->DriveCartesian(joystick_x, joystick_x, 0.0, 
                            units::make_unit<units::degree_t>(gyro));
        
        _setOutputs();
    }
    else
    {
        this->StopMotors();
    }    
}

void Drivetrain::DriveAuto(double magnitude, double angle, double heading, bool shift)
{
    if(shifter != nullptr)
        shifter->Set(shift);

    if(md != nullptr)
    {
        md->SetMaxWheelSpeed(shift ? C_LOW_GEAR_MAX_SPEED : C_HIGH_GEAR_MAX_SPEED);

        md->DrivePolar(magnitude,
                       units::make_unit<units::degree_t>(angle), 
                       heading);

        _setOutputs();
    }
}

void Drivetrain::DriveDirect(double rawFR, double rawFL, double rawBR, double rawBL)
{
    if(FR != nullptr) { FR->Set(ControlMode::PercentOutput, F_Limit(-1.0, 1.0, rawFR)); }
    if(FL != nullptr) { FL->Set(ControlMode::PercentOutput, F_Limit(-1.0, 1.0, rawFL)); }
    if(BR != nullptr) { BR->Set(ControlMode::PercentOutput, F_Limit(-1.0, 1.0, rawBR)); }
    if(BL != nullptr) { BL->Set(ControlMode::PercentOutput, F_Limit(-1.0, 1.0, rawBL)); }
}

void Drivetrain::StopMotors()
{
    if(FR != nullptr) { FR->Set(ControlMode::PercentOutput, 0.0); }
    if(FL != nullptr) { FL->Set(ControlMode::PercentOutput, 0.0); }
    if(BR != nullptr) { BR->Set(ControlMode::PercentOutput, 0.0); }
    if(BL != nullptr) { BL->Set(ControlMode::PercentOutput, 0.0); }
}

void Drivetrain::_setOutputs()
{
    if(md != nullptr)
    {
        if(FR != nullptr) { FR->Set(ControlMode::PercentOutput, md->GetWheelOutput(FRONT_RIGHT)); }
        if(FL != nullptr) { FL->Set(ControlMode::PercentOutput, md->GetWheelOutput(FRONT_LEFT)); }
        if(BR != nullptr) { BR->Set(ControlMode::PercentOutput, md->GetWheelOutput(REAR_RIGHT)); }
        if(BL != nullptr) { BL->Set(ControlMode::PercentOutput, md->GetWheelOutput(REAR_LEFT)); }
    }
}