// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "Constants.h"

RobotContainer::RobotContainer() : m_autonomousCommand(&m_subsystem) {
  // Initialize all of your commands and subsystems here
  dt = new Drivetrain(std::make_tuple<int, int>(C_SRX_FR_CIM, C_SRX_FR_MINI),
                      std::make_tuple<int, int>(C_SRX_FL_CIM, C_SRX_FL_MINI), 
                      std::make_tuple<int, int>(C_SRX_BR_CIM, C_SRX_BR_MINI), 
                      std::make_tuple<int, int>(C_SRX_BL_CIM, C_SRX_BL_MINI), 
                      C_DRIVE_SOL);



  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &m_autonomousCommand;
}
