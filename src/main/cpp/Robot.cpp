// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "subsystems/X22/X22_Constants.h"

#include "FRC3484_Lib/utils/SC_Functions.h"


#include "frc/PneumaticsModuleType.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

using namespace SC;

void Robot::RobotInit() 
{
	GP1_Driver = new XboxController(C_DRIVER_USB);
	BB_GameDevice = new GenericHID(C_GAMEDEV_USB);

#ifdef CLIMB_CONTROL_SEPARATE
	BB_Climb = new GenericHID(C_CLIMB_USB);
#endif

  	x22_drive = new X22_Drivetrain(C_X22_TRACK_WIDTH, C_HI_GEAR_MAX_SPEED, 90_deg_per_s,
                                   std::make_tuple<int, int>(C_FX_LEFT_MASTER, C_FX_LEFT_SLAVE),
                                   std::make_tuple<int, int>(C_FX_RIGHT_MASTER, C_FX_RIGHT_SLAVE),
                                   SC::SC_DoubleSolenoid{C_PCM, frc::PneumaticsModuleType::REVPH, C_DRIVE_SOL, 1});

	x22_intake = new X22_Intake(C_SPX_INTAKE, 
								C_SPX_FEED_MASTER, C_SPX_FEED_SLAVE, 
								SC::SC_Solenoid{C_PCM, frc::PneumaticsModuleType::REVPH, C_SOL_INTAKE},
								C_DI_CH_PROX_SEN, frc::I2C::Port::kOnboard);

	x22_launcher = new X22_Launcher(C_FX_LAUNCH_1, C_FX_LAUNCH_2, C_SPX_TURRET,
									C_DI_CH_TURRET_LS_MIN, C_DI_CH_TURRET_LS_MAX,
									SC::SC_Solenoid{C_PCM, frc::PneumaticsModuleType::REVPH, C_SOL_LOADER},
									SC::SC_Solenoid{C_PCM, frc::PneumaticsModuleType::REVPH, C_SOL_LAUNCH_ANGLE});

	Throttle_Range_Normal(-C_DRIVE_MAX_DEMAND, C_DRIVE_MAX_DEMAND);
	Throttle_Range_Fine(-C_DRIVE_MAX_DEMAND_FINE, C_DRIVE_MAX_DEMAND_FINE);
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
	/*======================*/
	/*====Driver Controls===*/
	/*======================*/

	forceLowGear = _HandleDriverInputs();

	//GP1_Driver->GetAButton() || GP1_Driver->GetRightBumper();

#ifdef DRIVE_MODE_TANK
	x22_drive->Drive_Tank(SC::F_Deadband(rightDemand, C_DRIVE_DEADBAND),
						  SC::F_Deadband(leftDemand, C_DRIVE_DEADBAND),
						  forceLowGear);
#elif defined(DRIVE_MODE_ARCADE)
	x22_drive->Drive_Arcade(SC::F_Deadband(throttleDemand, C_DRIVE_DEADBAND),
					 		SC::F_Deadband(turnDemand, C_DRIVE_DEADBAND),
                   	 		forceLowGear);

#elif defined(DRIVE_MODE_CURVE)
	x22_drive->Drive_Curve(SC::F_Deadband(throttleDemand, C_DRIVE_DEADBAND),
							SC::F_Deadband(turnDemand, C_DRIVE_DEADBAND),
							forceLowGear);
#endif

	/*===========================*/
	/*====Game Device Controls===*/
	/*===========================*/

	/*
	 * Button 1 - Normal intake operation
	 * Button 3 - Force Feed
	 * Button 4 - Reverse Feed
     */
	x22_intake->Collect(BB_GameDevice->GetRawButton(C_GD_INTAKE), BB_GameDevice->GetRawButton(C_GD_FORCE_FEED), BB_GameDevice->GetRawButton(C_GD_FEED_EJECT));

	x22_launcher->Periodic(BB_GameDevice->GetRawButton(C_GD_LAUNCH_AUTO) && x22_intake->IsCargoLoaded(), // Auto Functionality
						   BB_GameDevice->GetRawAxis(C_GD_TURRET_OR_AXIS), BB_GameDevice->GetRawButton(C_GD_LAUNCH_FORCE), BB_GameDevice->GetRawButton(C_GD_LAUNCH_EJECT));

	x22_climb->Periodic();
}

bool Robot::_HandleDriverInputs()
{

	bool __inLowGear = false;

#ifdef DRIVE_MODE_TANK
	throttleDemand = F_Scale(-100.0, 100.0, Throttle_Range_Normal, -GP1_Driver->GetRawAxis(C_DRIVER_THROTTLE_AXIS));
	turnDemand = F_Scale(-100.0, 100.0, Throttle_Range_Normal, GP1_Driver->GetRawAxis(C_DRIVER_THROTTLE_AXIS));
#elif defined(DRIVE_MODE_ARCADE) || defined(DRIVE_MODE_CURVE)
#ifdef DRIVER_FINE_ADJ_MODE
	if(GP1_Driver->GetRawButton(C_DRIVE_ADJ_BTN))
	{
		// Fine control mode; Scales driver input to smaller range for finer control
		throttleDemand = F_Scale(-100.0, 100.0, Throttle_Range_Fine, -GP1_Driver->GetRawAxis(C_DRIVER_STEER_AXIS));
		turnDemand = F_Scale(-100.0, 100.0, Throttle_Range_Fine, GP1_Driver->GetRawAxis(C_DRIVER_THROTTLE_AXIS));

		__inLowGear = true;
	}
	else
	{
#endif
		// Normal control mode
		throttleDemand = F_Scale(-100.0, 100.0, Throttle_Range_Normal, -GP1_Driver->GetRawAxis(C_DRIVER_THROTTLE_AXIS));
		turnDemand = F_Scale(-100.0, 100.0, Throttle_Range_Normal, GP1_Driver->GetRawAxis(C_DRIVER_THROTTLE_AXIS));
		__inLowGear = GP1_Driver->GetRawButton(C_DRIVER_SHIFT_LOW_BTN);
#ifdef DRIVER_FINE_ADJ_MODE
	}
#endif // DRIVER_FINE_ADJ_MODE
#endif // DRIVE_MODE

	return __inLowGear;
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
