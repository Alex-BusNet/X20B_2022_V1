#include "subsystems/X22/X22_Intake.h"

#include "Constants.h"

using namespace SC;
using namespace frc;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

X22_Intake::X22_Intake(int IntakeID, int FeedID_Master, int FeedID_Slave, SC_Solenoid Sol, int ProxSen_Ch)
{
    Motor_Intake_Master = new VictorSPX(IntakeID);
    Motor_Intake_Master->SetNeutralMode(NeutralMode::Coast);
    Motor_Intake_Master->ConfigOpenloopRamp(C_INTAKE_RAMP_TIME);

    Motor_Feed_Master = new VictorSPX(FeedID_Master);
    Motor_Feed_Master->SetNeutralMode(NeutralMode::Brake);
    
    Motor_Feed_Slave = new VictorSPX(FeedID_Slave);
    Motor_Feed_Slave->SetNeutralMode(NeutralMode::Brake);
    Motor_Feed_Slave->Follow(*Motor_Feed_Master);
    
    Sol_1 = new Solenoid(Sol.CtrlID, Sol.CtrlType, Sol.Channel);
    Sol_1->Set(false);

    DI_Prox = new DigitalInput(ProxSen_Ch);

}

X22_Intake::~X22_Intake()
{
    if(Motor_Intake_Master != NULL) { delete Motor_Intake_Master; }
    if(Motor_Feed_Master != NULL) { delete Motor_Feed_Master; }
    if(Motor_Feed_Slave != NULL) { delete Motor_Feed_Slave; }

    if(Sol_1 != NULL) { delete Sol_1; }
    if(DI_Prox != NULL) { delete DI_Prox; }
}

void X22_Intake::Collect(bool Run)
{
    // Deploy the intake and run the motor when commanded.
    Sol_1->Set(Run);
    Motor_Intake_Master->Set(ControlMode::PercentOutput, (Run ? C_INTAKE_DRIVE_SPEED : 0.0));

    // Run the feeder when the Prox sensor detects there is no cargo loaded, and there is one in the feeder.
    Motor_Feed_Master->Set(ControlMode::PercentOutput, (DI_Prox->Get() ? C_FEED_DRIVE_SPEED : 0.0));
}