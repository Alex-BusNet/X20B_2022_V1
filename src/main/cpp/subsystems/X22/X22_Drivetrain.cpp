#include "subsystems/X22/X22_Drivetrain.h"
#include "Constants.h"

using namespace SC;
using namespace frc;
using namespace units::length;
using namespace units::velocity;
using namespace units::angular_velocity;
using namespace ctre::phoenix::motorcontrol::can;
using namespace ctre::phoenix::motorcontrol;

X22_Drivetrain::X22_Drivetrain()
{
    drive = new SC_DifferentialDrive(1_in, 1_fps, 1_deg_per_s);
}

X22_Drivetrain::X22_Drivetrain(inch_t trackwidth, feet_per_second_t MaxTangVel, degrees_per_second_t MaxRotVel,
                               std::tuple<int, int> LeftIDs, std::tuple<int, int> RightIDs, SC_Solenoid Shifter)
{
    drive = new SC_DifferentialDrive(trackwidth, MaxTangVel, MaxRotVel);
    
    _linVelRange(-MaxTangVel.value(), MaxTangVel.value());
    _angVelRange(-MaxRotVel.value(), MaxRotVel.value());

    int sCh = -1;

    // Initialize front right wheel and slave controller
    if(LeftIDs != C_BLANK_IDS) 
    { 
        Motor_Left_Master = new TalonFX(std::get<0>(LeftIDs));
        sCh = std::get<1>(LeftIDs);

        if(sCh > -1)
        {
            Motor_Left_Slave = new TalonFX(sCh);
            Motor_Left_Slave->Follow(*Motor_Left_Master);
        }
        else
        {
            Motor_Left_Slave = nullptr;
        }
    } 
    else
    {
        Motor_Left_Master = nullptr;
        Motor_Left_Slave = nullptr;
    }
    

    // Initialize front left wheel and slave controller
     if(RightIDs != C_BLANK_IDS) 
    { 
        Motor_Right_Master = new TalonFX(std::get<0>(RightIDs));
        sCh = std::get<1>(RightIDs);

        if(sCh > -1)
        {
            Motor_Right_Slave = new TalonFX(sCh);
            Motor_Right_Slave->Follow(*Motor_Right_Master);
        }
        else
        {
            Motor_Right_Slave = nullptr;
        }
    } 
    else
    {
        Motor_Right_Master = nullptr;
        Motor_Right_Slave = nullptr;
    }
    
    if(Shifter.Channel != -1) { _shifter = new Solenoid(Shifter.CtrlID, Shifter.CtrlType, Shifter.Channel); }
    else { _shifter = nullptr; }

}

void X22_Drivetrain::Drive(double Throttle, double Rotation)
{
	if(drive != NULL)
	{
		drive->Drive_Vel(F_Scale(0.0, 100.0, _linVelRange.Val_min, _linVelRange.Val_max, Throttle),
			   			 F_Scale(0.0, 100.0, _angVelRange.Val_min, _angVelRange.Val_max, Rotation),
                         0, true);

		if(Motor_Left_Master != NULL) 
        {
            // TODO: Auto-shifting

            Motor_Left_Master->Set(ControlMode::PercentOutput, drive->GetWheelOutput(LEFT_WHEEL)); 
        }

   	    if(Motor_Right_Master != NULL) { Motor_Right_Master->Set(ControlMode::PercentOutput, drive->GetWheelOutput(RIGHT_WHEEL)); }
    }
}