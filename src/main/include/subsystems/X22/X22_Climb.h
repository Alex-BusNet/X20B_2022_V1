#ifndef X22_CLIMB_H
#define X22_CLIMB_H

#include "FRC3484_Lib/utils/SC_Datatypes.h"
#include "FRC3484_Lib/utils/TC3_Syntax.h"

#include "ctre/phoenix/motorcontrol/can/TalonSRX.h"

#include "frc/Solenoid.h"
#include "frc/filter/Debouncer.h"

class X22_Climb
{
public:
	X22_Climb(SC::SC_Solenoid Stage1_Ext, SC::SC_Solenoid Stage1_Grab,
			 SC::SC_Solenoid Stage2_Ext, SC::SC_Solenoid Stage2_Grab,
			 std::tuple<int, int> Stage2_AdjustID);

	~X22_Climb();

	void Periodic(bool Stage1_Ext, bool Stage1_Claw, bool Stage2_Ext, bool Stage2_Claw);

	void Step_Forward();
	void Step_Reverse();

	void ResetSequence();

	void SetSequence(int val); 

private:
	int _state, _nextstate, _prevstate;


	frc::Solenoid *_stage1_ext, *_stage1_grab;
	frc::Solenoid *_stage2_ext, *_stage2_grab;

	ctre::phoenix::motorcontrol::can::TalonSRX *_motor_adj_master, *_motor_adj_slave;

	bool stage1_ext_state, stage1_claw_state, stage2_ext_state, stage2_claw_state;

	R_TRIG *rTrig_s1e, *rTrig_s1c, *rTrig_s2e, *rTrig_s2c;
	frc::Debouncer *_dbnc_s1e, *_dbnc_s1c, *_dbnc_s2e, *_dbnc_s2c;

};


#endif // X22_CLIMB_H
