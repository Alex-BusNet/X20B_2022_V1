#include "subsystems/X22/X22_Drivetrain.h"
#include "Constants.h"

#include "units/math.h"

#include "frc/shuffleboard/BuiltInWidgets.h"

#include "frc/smartdashboard/SmartDashboard.h"

using namespace SC;
using namespace frc;
using namespace units::length;
using namespace units::velocity;
using namespace units::angular_velocity;
using namespace ctre::phoenix::motorcontrol::can;
using namespace ctre::phoenix::motorcontrol;

X22_Drivetrain::X22_Drivetrain(inch_t trackwidth, feet_per_second_t MaxTangVel, degrees_per_second_t MaxRotVel,
                               std::tuple<int, int> LeftIDs, std::tuple<int, int> RightIDs, SC_DoubleSolenoid Shifter)
{
    // drive = new SC_DifferentialDrive(trackwidth, MaxTangVel, MaxRotVel);

    ntLeftOut = Shuffleboard::GetTab("X22")
                    .Add("Left Output", 0.0)
                    .WithWidget(BuiltInWidgets::kNumberBar)
                    .WithProperties(props_numbar)
                    .GetEntry();
    
    ntRightOut = Shuffleboard::GetTab("X22")
                    .Add("Right Output", 0.0)
                    .WithWidget(BuiltInWidgets::kNumberBar)
                    .WithProperties(props_numbar)
                    .GetEntry();

	ntLeftVel = Shuffleboard::GetTab("X22")
                    .Add("Left Velocity", 0.0)
                    .WithWidget(BuiltInWidgets::kNumberBar)
                    .WithProperties(props_diffdrive_hi)
                    .GetEntry();
    
    ntRightVel = Shuffleboard::GetTab("X22")
                    .Add("Right Velocity", 0.0)
                    .WithWidget(BuiltInWidgets::kNumberBar)
                    .WithProperties(props_diffdrive_hi)
                    .GetEntry();

	ntLeftEnc_Raw = Shuffleboard::GetTab("X22")
                    .Add("Left Encoder - Sensor", 0.0)
                    .WithWidget(BuiltInWidgets::kNumberBar)
                    .WithProperties(props_enc_values)
                    .GetEntry();
    
    ntRightEnc_Raw = Shuffleboard::GetTab("X22")
                    .Add("Right Encoder - Sensor", 0.0)
                    .WithWidget(BuiltInWidgets::kNumberBar)
                    .WithProperties(props_enc_values)
                    .GetEntry();

	ntChassisVx = Shuffleboard::GetTab("X22")
                    .Add("Chassis Vx", 0.0)
                    .WithWidget(BuiltInWidgets::kNumberBar)
                    .WithProperties(props_diffdrive_hi)
                    .GetEntry();
    
    ntChassisVy = Shuffleboard::GetTab("X22")
                    .Add("Chassis Vy", 0.0)
                    .WithWidget(BuiltInWidgets::kNumberBar)
                    .WithProperties(props_diffdrive_hi)
                    .GetEntry();

    ntChassisOmega = Shuffleboard::GetTab("X22")
                    .Add("Chassis ω", 0.0)
                    .WithWidget(BuiltInWidgets::kNumberBar)
                    .WithProperties(props_diffdrive_hi)
                    .GetEntry();

    ntThrottleIn = Shuffleboard::GetTab("X22")
                    .Add("Throttle Input", 0.0)
                    .WithWidget(BuiltInWidgets::kNumberBar)
                    .WithProperties(props_numbar)
                    .GetEntry();

    ntRotationIn = Shuffleboard::GetTab("X22")
                    .Add("Rotation Input", 0.0)
                    .WithWidget(BuiltInWidgets::kNumberBar)
                    .WithProperties(props_numbar)
                    .GetEntry();
    
    ntDriveValid = Shuffleboard::GetTab("X22")
                    .Add("Drive Valid", 0.0)
                    .WithWidget(BuiltInWidgets::kBooleanBox)
                    .GetEntry();

    ntInLowGear = Shuffleboard::GetTab("X22")
                    .Add("In Low Gear", 0.0)
                    .WithWidget(BuiltInWidgets::kBooleanBox)
                    .GetEntry();

    ntInHighGear = Shuffleboard::GetTab("X22")
                    .Add("In High Gear", 0.0)
                    .WithWidget(BuiltInWidgets::kBooleanBox)
                    .GetEntry();
    
    ntRampTime = Shuffleboard::GetTab("X22")
                    .Add("DT Ramp Time", 2.0)
                    .GetEntry();

	ntLowSF = Shuffleboard::GetTab("X22")
                    .Add("Low Gear Scale Factor", C_DT_SCALE_FACTOR_LO)
					.WithWidget(BuiltInWidgets::kTextView)
					.GetEntry();

	ntHighSF = Shuffleboard::GetTab("X22")
                    .Add("High Gear Scale Factor", C_DT_SCALE_FACTOR_HI)
					.WithWidget(BuiltInWidgets::kTextView)
					.GetEntry();

    _linVelRange(-MaxTangVel.value(), MaxTangVel.value());
    _angVelRange(-MaxRotVel.value(), MaxRotVel.value());

    int sCh = -1;

    // Initialize front right wheel and slave controller
    if(LeftIDs != C_BLANK_IDS) 
    { 
        Motor_Left_Master = new WPI_TalonFX(std::get<0>(LeftIDs));
        Motor_Left_Master->SetInverted(true);
        Motor_Left_Master->SetNeutralMode(NeutralMode::Coast);
        Motor_Left_Master->ConfigOpenloopRamp(2);
		Motor_Left_Master->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
		Motor_Left_Master->SetSelectedSensorPosition(0);

        sCh = std::get<1>(LeftIDs);

        if(sCh > -1)
        {
            Motor_Left_Slave = new WPI_TalonFX(sCh);
            Motor_Left_Slave->Follow(*Motor_Left_Master);
			Motor_Left_Slave->SetInverted(true);
			Motor_Left_Slave->SetNeutralMode(NeutralMode::Coast);
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
        Motor_Right_Master = new WPI_TalonFX(std::get<0>(RightIDs));
        Motor_Left_Master->SetInverted(false);
        Motor_Right_Master->SetNeutralMode(NeutralMode::Coast);
        Motor_Right_Master->ConfigOpenloopRamp(2);
		Motor_Right_Master->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
		Motor_Right_Master->SetSelectedSensorPosition(0);

        sCh = std::get<1>(RightIDs);

        if(sCh > -1)
        {
            Motor_Right_Slave = new WPI_TalonFX(sCh);
            Motor_Right_Slave->Follow(*Motor_Right_Master);
			Motor_Right_Slave->SetInverted(false);
			Motor_Right_Slave->SetNeutralMode(NeutralMode::Coast);
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
    
    drive = new DifferentialDrive(*Motor_Left_Master, *Motor_Right_Master);
	ddKinematics = new DifferentialDriveKinematics(X22_TRACK_WIDTH);

    if(Shifter.Fwd_Channel != -1) 
	{ 
		_shifter = new DoubleSolenoid(Shifter.CtrlID, Shifter.CtrlType, Shifter.Fwd_Channel, Shifter.Rev_Channel);
		Shift(true, false);
	}
    else { _shifter = nullptr; }
}

X22_Drivetrain::~X22_Drivetrain()
{
	if(drive != NULL) { delete drive; }
	if(ddKinematics != NULL) { delete ddKinematics; }
	if(Motor_Left_Master != NULL) { delete Motor_Left_Master; }
	if(Motor_Left_Slave != NULL) { delete Motor_Left_Slave; }
	if(Motor_Right_Master != NULL) { delete Motor_Right_Master; }
	if(Motor_Right_Slave != NULL) { delete Motor_Right_Slave; }
	if(_shifter != NULL) { delete _shifter; }

	ntDriveValid.Delete();
	ntLeftOut.Delete();
	ntRightOut.Delete();
	ntThrottleIn.Delete();
	ntRotationIn.Delete();
	ntInLowGear.Delete();
	ntInHighGear.Delete();
	ntRampTime.Delete();
	ntLeftVel.Delete();
	ntRightVel.Delete();
	ntChassisVy.Delete();
	ntChassisVx.Delete();
	ntChassisOmega.Delete();
	ntLeftEnc_Raw.Delete();
	ntRightEnc_Raw.Delete();
	ntLowSF.Delete();
	ntHighSF.Delete();
}

void X22_Drivetrain::Drive(double Throttle, double Rotation, bool ShiftOverride)
{
	ntThrottleIn.SetDouble(Throttle);
	ntRotationIn.SetDouble(Rotation);

	if(Motor_Left_Master != NULL) { Motor_Left_Master->ConfigOpenloopRamp(ntRampTime.GetDouble(2.0)); }
	if(Motor_Right_Master != NULL) { Motor_Right_Master->ConfigOpenloopRamp(ntRampTime.GetDouble(2.0)); }

	if(drive != NULL)
	{
	   	// Convert driver inputs to drivetrain outputs
		double scaledThrottle = std::copysign(std::min((Throttle*Throttle)*2.0, std::abs(Throttle)), Throttle);
		wsInput = drive->ArcadeDriveIK(scaledThrottle, -Rotation, inLowGear);

		ws_PV.left = units::make_unit<feet_per_second_t>(_CalcWheelVelocity(Motor_Left_Master->GetSelectedSensorVelocity(0)));
		ws_PV.right = units::make_unit<feet_per_second_t>(_CalcWheelVelocity(Motor_Right_Master->GetSelectedSensorVelocity(0)));

		cs_PV = ddKinematics->ToChassisSpeeds(ws_PV);

		// Send subsystem data to the dashboard
		ntLeftOut.SetDouble(wsInput.left);
		ntRightOut.SetDouble(wsInput.right);

		ntLeftEnc_Raw.SetDouble(Motor_Left_Master->GetSelectedSensorVelocity(0));
		ntRightEnc_Raw.SetDouble(Motor_Right_Master->GetSelectedSensorVelocity(0));

		ntLeftVel.SetDouble(units::convert<meters_per_second, feet_per_second>(ws_PV.left).value());
		ntRightVel.SetDouble(units::convert<meters_per_second, feet_per_second>(ws_PV.right).value());
		ntChassisVx.SetDouble(units::convert<meters_per_second, feet_per_second>(cs_PV.vx).value());
		ntChassisVy.SetDouble(units::convert<meters_per_second, feet_per_second>(cs_PV.vy).value());
		ntChassisOmega.SetDouble(units::convert<radians_per_second, degrees_per_second>(cs_PV.omega).value());

		// Auto-shifting logic
        Shift(ShiftOverride || (units::math::abs(cs_PV.vx) < C_SHIFT_DOWN_SPEED),
			 !ShiftOverride && (units::math::abs(cs_PV.vx) > C_SHIFT_UP_SPEED));

		// Set motor outputs
		Motor_Left_Master->Set(ControlMode::PercentOutput, wsInput.left);
		Motor_Right_Master->Set(ControlMode::PercentOutput, wsInput.right);

		// Handle motor safety
        drive->Feed();
        drive->FeedWatchdog();
    }
}

void X22_Drivetrain::Shift(bool ShiftLow, bool ShiftHigh)
{
    if(_shifter != NULL)
    {
        _shifter->Set((ShiftHigh && !ShiftLow) ? DoubleSolenoid::kForward : DoubleSolenoid::kReverse);
    }

	inLowGear = ShiftLow;
	inHighGear = ShiftHigh;
	
    ntInLowGear.SetBoolean(inLowGear);
	ntInHighGear.SetBoolean(inHighGear);
}

double X22_Drivetrain::_CalcWheelVelocity(double counts)
{
	if(this->inLowGear) { return counts * C_DT_SCALE_FACTOR_LO; }
	else if (this->inHighGear) { return counts * C_DT_SCALE_FACTOR_HI; }
	else { return 0; }
}