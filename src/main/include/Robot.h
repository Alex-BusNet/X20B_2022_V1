// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>

#include "RobotContainer.h"

#include <frc/XboxController.h>
#include "subsystems/Drivetrain.h"
#include "FRC3484_Lib/utils/SC_Datatypes.h"


#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableValue.h>

using namespace frc;
using namespace ctre::phoenix::motorcontrol::can;

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

 private:
  // Have it null by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  frc2::Command* m_autonomousCommand = nullptr;

  RobotContainer m_container;
  Drivetrain *drivetrain;

  XboxController  *GP1_Driver, *GP2_GameDevice; // GP = Gamepad

  double motorOutput;
  double ctrl_1_A0;
};
