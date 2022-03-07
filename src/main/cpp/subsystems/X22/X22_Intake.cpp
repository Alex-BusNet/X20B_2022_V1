#include "subsystems/X22/X22_Intake.h"

#include "Constants.h"

using namespace SC;
using namespace frc;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

X22_Intake::X22_Intake(int IntakeID, int FeedID_Master, int FeedID_Slave, SC_Solenoid Sol, int ProxSen_Ch, I2C::Port ColorSenPort)
{
    Motor_Intake_Master = new VictorSPX(IntakeID);
    Motor_Intake_Master->SetNeutralMode(NeutralMode::Brake);
    Motor_Intake_Master->ConfigOpenloopRamp(C_INTAKE_RAMP_TIME);

    Motor_Feed_Master = new VictorSPX(FeedID_Master);
    Motor_Feed_Master->SetNeutralMode(NeutralMode::Brake);
    
    Motor_Feed_Slave = new VictorSPX(FeedID_Slave);
    Motor_Feed_Slave->SetNeutralMode(NeutralMode::Brake);
    Motor_Feed_Slave->Follow(*Motor_Feed_Master);
    
    Sol_1 = new Solenoid(Sol.CtrlID, Sol.CtrlType, Sol.Channel);
    Sol_1->Set(false);

    DI_Prox = new DigitalInput(ProxSen_Ch);

	this->_sen_loader = new SC_ColorSensor(ColorSenPort);

}

X22_Intake::~X22_Intake()
{
    if(Motor_Intake_Master != NULL) { delete Motor_Intake_Master; }
    if(Motor_Feed_Master != NULL) { delete Motor_Feed_Master; }
    if(Motor_Feed_Slave != NULL) { delete Motor_Feed_Slave; }

    if(Sol_1 != NULL) { delete Sol_1; }
    if(DI_Prox != NULL) { delete DI_Prox; }
}

void X22_Intake::Collect(bool Run, bool ForceFeed, bool ReverseFeed)
{
    double __feedOut, __intakeOut;

    /* 
     * Deploy the intake and run the motor when commanded.
     * When using ReverseFeed, the intake touches the cargo
     * stored in the feeder, which will aid in the removal of the cargo.
     */
    Sol_1->Set(Run && !ReverseFeed);

    /* Intake Motor operation logic */
    if(ReverseFeed) { __intakeOut = -C_INTAKE_DRIVE_SPEED; } // Reverse the intake motor direction
    else if(Run) { __intakeOut = C_INTAKE_DRIVE_SPEED; } // Normal intake operation
    else { __intakeOut = 0.0; } // Intake motor idle

    /* Feed Motor operation logic */
    if(ReverseFeed) { __feedOut = -C_FEED_DRIVE_SPEED; } // Reverse the feed motors
    else if((!this->IsCargoLoaded() && this->IsCargoStored()) || ForceFeed) { __feedOut = C_FEED_DRIVE_SPEED; } // Normal feeder operation
    else { __feedOut = 0.0; } // Feed motor idle

    // Set motor outputs
    Motor_Intake_Master->Set(ControlMode::PercentOutput, __intakeOut);
    Motor_Feed_Master->Set(ControlMode::PercentOutput, __feedOut);
}

bool X22_Intake::IsCargoLoaded()
{
    return this->_sen_loader->GetDistance() > 1500;
}

bool X22_Intake::IsCargoStored()
{
    this->DI_Prox->Get();
}