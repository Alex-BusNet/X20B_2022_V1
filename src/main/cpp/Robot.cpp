// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include "FRC3484_Lib/utils/SC_Functions.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

void Robot::RobotInit() 
{
  GP1_Driver = new XboxController(/*USB Port*/ 0);
  GP2_GameDevice = new XboxController(/*USB Port*/ 1);

  drivetrain = new Drivetrain(std::make_tuple<int, int>(C_SRX_FR_CIM, C_SRX_FR_MINI),
                              std::make_tuple<int, int>(C_SRX_FL_CIM, C_SRX_FL_MINI),
                              std::make_tuple<int, int>(C_SRX_BR_CIM, C_SRX_BR_MINI),
                              std::make_tuple<int, int>(C_SRX_BL_CIM, C_SRX_BL_MINI),
                              C_DRIVE_SOL);
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
	drivetrain->Drive(SC::F_Deadband(GP1_Driver->GetLeftX(), C_DRIVE_DEADBAND),
						SC::F_Deadband(GP1_Driver->GetLeftY(), C_DRIVE_DEADBAND),
						0.0, GP1_Driver->GetAButton());
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
