#ifndef X22_CLIMB_H
#define X22_CLIMB_H

#include "FRC3484_Lib/utils/SC_Datatypes.h"

#include "ctre/phoenix/motorcontrol/can/TalonSRX.h"

#include "frc/DoubleSolenoid.h"

class X22_Climb
{
public:
	X22_Climb(SC::SC_DoubleSolenoid Stage1_Ext, SC::SC_DoubleSolenoid Stage1_Grab,
			 SC::SC_DoubleSolenoid Stage2_Ext, SC::SC_DoubleSolenoid Stage2_Grab,
			 std::tuple<int, int> Stage2_AdjustID);

	~X22_Climb();

	void Periodic();

	void Step_Forward();
	void Step_Reverse();

	void ResetSequence();

private:
	int _state, _nextState, _prevState;

	frc::DoubleSolenoid *_stage1_ext, *_stage1_grab;
	frc::DoubleSolenoid *_stage2_ext, *_stage2_grab;

	ctre::phoenix::motorcontrol::can::TalonSRX *_motor_adj_master, *_motor_adj_slave;
};


#endif // X22_CLIMB_H