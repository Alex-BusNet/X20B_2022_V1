#include "subsystems/X22/X22_Climb.h"
#include "Constants.h"

using namespace SC;
using namespace std;
using namespace frc;
using namespace ctre::phoenix::motorcontrol::can;
using namespace ctre::phoenix::motorcontrol;

X22_Climb::X22_Climb(SC_DoubleSolenoid Stage1_Ext, SC_DoubleSolenoid Stage1_Grab, SC_DoubleSolenoid Stage2_Ext, SC_DoubleSolenoid Stage2_Grab, tuple<int, int> Stage2_AdjustID)
{
	this->_stage1_ext  = new DoubleSolenoid(Stage1_Ext.CtrlID,  Stage1_Ext.CtrlType,  Stage1_Ext.Fwd_Channel,  Stage1_Ext.Rev_Channel);
	this->_stage1_grab = new DoubleSolenoid(Stage1_Grab.CtrlID, Stage1_Grab.CtrlType, Stage1_Grab.Fwd_Channel, Stage1_Grab.Rev_Channel);
	this->_stage2_ext  = new DoubleSolenoid(Stage2_Ext.CtrlID,  Stage2_Ext.CtrlType,  Stage2_Ext.Fwd_Channel,  Stage2_Ext.Rev_Channel);
	this->_stage2_grab = new DoubleSolenoid(Stage2_Grab.CtrlID, Stage2_Grab.CtrlType, Stage2_Grab.Fwd_Channel, Stage2_Grab.Rev_Channel);

	// Set default position of solenoids
	this->_stage1_ext->Set(DoubleSolenoid::kReverse);
	this->_stage1_grab->Set(DoubleSolenoid::kReverse);
	this->_stage2_ext->Set(DoubleSolenoid::kReverse);
	this->_stage2_grab->Set(DoubleSolenoid::kReverse);

	/*
		NOTE: One of these motors probably needs to be inverted.
	*/

	if(Stage2_AdjustID != C_BLANK_IDS) // Check if the adjustment motors are being used.
	{
		this->_motor_adj_master = new TalonSRX(std::get<0>(Stage2_AdjustID));

		if(this->_motor_adj_master != NULL) // Check if the master motor is valid
		{
			this->_motor_adj_master->SetNeutralMode(NeutralMode::Brake); // Set to brake mode

			if(std::get<1>(Stage2_AdjustID) != -1) // Check if slave motor is being used
			{
				this->_motor_adj_slave = new TalonSRX(std::get<1>(Stage2_AdjustID));
				
				if(this->_motor_adj_slave != NULL) // Check if slave is valid
				{
					this->_motor_adj_slave->SetNeutralMode(NeutralMode::Brake); // Set to brake mode
					this->_motor_adj_slave->Follow(*this->_motor_adj_master); // Set to follow master.
				}
			}
			else
			{
				this->_motor_adj_slave = NULL;
			}
		}
		else
		{
			this->_motor_adj_slave = NULL;
		}
	}
	else
	{
		this->_motor_adj_master = NULL;
		this->_motor_adj_slave = NULL;
	}
}

X22_Climb::~X22_Climb()
{
	if(this->_stage1_ext != NULL) { delete this->_stage1_ext; }
	if(this->_stage1_grab != NULL) { delete this->_stage1_grab; }
	if(this->_stage2_ext != NULL) { delete this->_stage2_ext; }
	if(this->_stage2_grab != NULL) { delete this->_stage2_grab; }

	if(this->_motor_adj_master != NULL) { delete this->_motor_adj_master; }
	if(this->_motor_adj_slave != NULL) { delete this->_motor_adj_slave; }
}

void X22_Climb::Periodic()
{
	/*
	 *	Climb sequence:
	 		0) Idle / start
	 		1) Stage 1 release & Stage 2 release
			2) Stage 1 extend
			3) Stage 1 grab
			4) Stage 1 retract
			5) Stage 2 extend
			6) Stage 2 grab
			7) Stage 1 release
		--- If stage 2 adjustment is present ---
			8) Stage 2 retract
			9) Stage 1 grab
			10) Stage 2 release
			11) Adjust stage 2
			12) Stage 2 extend
			13) Stage 2 grab
			14) Stage 1 release
	 */

	switch(this->_state)
	{
	case 0: /* Idle */
		// todo: set default state of solenoids?
		break;

	case 1: /* Stage 1 release & Stage 2 release */
		this->_stage1_grab->Set(DoubleSolenoid::kForward);
		this->_stage2_grab->Set(DoubleSolenoid::kForward);
		break;

	case 2: /* Stage 1 extend */


	case 3: /* Stage 1 grab */


	case 4: /* Stage 1 retract */


	case 5: /* Stage 2 extend */


	case 6: /* Stage 2 grab */


	case 7: /* Stage 1 release */

	default:
		break;
	}
}

void X22_Climb::Step_Forward()
{
	this->_nextState = this->_state + 1;
}

void X22_Climb::Step_Reverse()
{
	this->_nextState = this->_state - 1;
}

void X22_Climb::ResetSequence()
{
	this->_nextState = 0;
}

