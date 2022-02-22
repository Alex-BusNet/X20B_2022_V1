// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "Constants.h"

#include "FRC3484_Lib/utils/SC_Functions.h"


#include "frc/PneumaticsModuleType.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

using namespace SC;

void Robot::RobotInit() 
{
	GP1_Driver = new XboxController(/*USB Port*/ 0);
	GP2_GameDevice = new XboxController(/*USB Port*/ 1);

  	x22_drive = new X22_Drivetrain(C_X22_TRACK_WIDTH, C_HI_GEAR_MAX_SPEED, 90_deg_per_s,
                                   std::make_tuple<int, int>(C_FX_LEFT_MASTER, C_FX_LEFT_SLAVE),
                                   std::make_tuple<int, int>(C_FX_RIGHT_MASTER, C_FX_RIGHT_SLAVE),
                                   SC::SC_DoubleSolenoid{C_PCM, frc::PneumaticsModuleType::REVPH, C_DRIVE_SOL, 1});
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
	frc2::CommandScheduler::GetInstance().Run();
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
	m_autonomousCommand = m_container.GetAutonomousCommand();

	if (m_autonomousCommand != nullptr) {
		m_autonomousCommand->Schedule();
	}
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
	if (m_autonomousCommand != nullptr) {
		m_autonomousCommand->Cancel();
		m_autonomousCommand = nullptr;
	}

}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() 
{
	if(GP1_Driver->GetRightBumper())
	{
		// Fine control mode; Scales driver input to smaller range for finer control
		throttleDemand = F_Scale(-100.0, 100.0, Throttle_Range_Fine, -GP1_Driver->GetLeftY());
		turnDemand = F_Scale(-100.0, 100.0, Throttle_Range_Fine, GP1_Driver->GetLeftX());
	}
	else
	{
		// Normal control mode
		// throttleDemand = F_Scale(-100.0, 100.0, Throttle_Range_Normal, -GP1_Driver->GetLeftY());
		// turnDemand = F_Scale(-100.0, 100.0, Throttle_Range_Normal, GP1_Driver->GetLeftX());
		throttleDemand = -GP1_Driver->GetLeftY();
		turnDemand = GP1_Driver->GetLeftX();

	}

	forceLowGear = GP1_Driver->GetAButton() || GP1_Driver->GetRightBumper();

	x22_drive->Drive(SC::F_Deadband(throttleDemand, C_DRIVE_DEADBAND),
					 SC::F_Deadband(turnDemand, C_DRIVE_DEADBAND),
                   	 forceLowGear);


  
}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
