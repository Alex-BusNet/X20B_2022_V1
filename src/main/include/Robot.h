// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>

#include "FRC3484_Lib/utils/SC_Datatypes.h"

#include "RobotContainer.h"

#include "subsystems/X22/X22_Drivetrain.h"
#include "subsystems/X22/X22_Intake.h"
#include "subsystems/X22/X22_Launcher.h"

#include <frc/XboxController.h>

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
  X22_Drivetrain *x22_drive;
  X22_Intake *x22_intake;
  X22_Launcher *x22_launcher;

  SC::SC_Range<double> Throttle_Range_Normal = {-C_DRIVE_MAX_DEMAND, C_DRIVE_MAX_DEMAND};
  SC::SC_Range<double> Throttle_Range_Fine = {-C_DRIVE_MAX_DEMAND_FINE, C_DRIVE_MAX_DEMAND_FINE};

  double throttleDemand, turnDemand;
  double forceLowGear;

  frc::XboxController  *GP1_Driver;// GP = Gamepad
  frc::GenericHID      *BB_GameDevice;  
};
