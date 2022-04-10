#include "subsystems/X22/X22_Climb.h"
#include "subsystems/X22/X22_Constants.h"

using namespace SC;
using namespace std;
using namespace frc;
using namespace ctre::phoenix::motorcontrol::can;
using namespace ctre::phoenix::motorcontrol;

X22_Climb::X22_Climb(SC_Solenoid Stage1_Ext, SC_Solenoid Stage1_Grab, SC_Solenoid Stage2_Ext, SC_Solenoid Stage2_Grab, tuple<int, int> Stage2_AdjustID)
{
    this-> _stage1_ext = new Solenoid(Stage1_Ext.CtrlID, Stage1_Ext.CtrlType, Stage1_Ext.Channel);
    this-> _stage1_grab = new Solenoid(Stage1_Grab.CtrlID, Stage1_Grab.CtrlType, Stage1_Grab.Channel);
    //this-> _stage2_ext = new Solenoid(Stage2_Ext.CtrlID, Stage2_Ext.CtrlType, Stage2_Ext.Channel);
    //this-> _stage2_grab = new Solenoid(Stage2_Grab.CtrlID, Stage2_Grab.CtrlType, Stage2_Grab.Channel);

    this->stage1_ext_state = false;
    this->stage1_claw_state = false;

    this->_stage1_ext->Set(false);
    this->_stage1_grab->Set(false);
    //this->_stage2_ext->Set(false);
    //this->_stage2_grab->Set(false);

    this->rTrig_s1e = new R_TRIG();     this->rTrig_s1e->Check(false);
    this->rTrig_s1c = new R_TRIG();     this->rTrig_s1c->Check(false);
    //this->rTrig_s2e = new R_TRIG();     this->rTrig_s2e->Check(false);
    //this->rTrig_s2c = new R_TRIG();     this->rTrig_s2c->Check(false);

    this->_dbnc_s1e = new Debouncer(C_CLIMB_DBNC_TIME, Debouncer::kRising);
    this->_dbnc_s1c = new Debouncer(C_CLIMB_DBNC_TIME, Debouncer::kRising);
    // this->_dbnc_s2e = new Debouncer(C_CLIMB_DBNC_TIME, Debouncer::kRising);
    // this->_dbnc_s2c = new Debouncer(C_CLIMB_DBNC_TIME, Debouncer::kRising);

    /*
        NOTE: One of these motors probably needs to be inverted.
    */

   /* if(Stage2_AdjustID != C_BLANK_IDS)
    {
        this->_motor_adj_master = new TalonSRX(std::get<0>(Stage2_AdjustID));

        if(this->_motor_adj_master != NULL) // check if the master motor is valid
        {
            this->_motor_adj_master->SetNeutralMode(NeutralMode::Brake);

            if(std::get<1>(Stage2_AdjustID) != -1) // check if slave motor is being used
            {
                this->_motor_adj_slave = new TalonSRX(std::get<1>(Stage2_AdjustID));

                if(this->_motor_adj_slave != NULL) // check if slave is valid
                {
                    this->_motor_adj_slave->SetNeutralMode(NeutralMode::Brake);
                    this->_motor_adj_slave->Follow(*this->_motor_adj_master);
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
    }*/

}

X22_Climb::~X22_Climb()
{
    if(this-> _stage1_ext  != NULL) {delete this->_stage1_ext; }
    if(this-> _stage1_grab != NULL) {delete this->_stage1_grab; }
    // if(this-> _stage2_ext  != NULL) {delete this->_stage2_ext; }
    // if(this-> _stage2_grab != NULL) {delete this->_stage2_grab; }

    if(this->_motor_adj_master != NULL) {delete this-> _motor_adj_master; }
    if(this->_motor_adj_slave  != NULL) {delete this-> _motor_adj_slave;  }

    if(this->rTrig_s1e != NULL) { delete this->rTrig_s1e; }
    if(this->rTrig_s1c != NULL) { delete this->rTrig_s1c; }
    // if(this->rTrig_s2e != NULL) { delete this->rTrig_s2e; }
    // if(this->rTrig_s2c != NULL) { delete this->rTrig_s2c; }

    if(this->_dbnc_s1e != NULL) { delete this->_dbnc_s1e; }
    if(this->_dbnc_s1c != NULL) { delete this->_dbnc_s1c; }
    // if(this->_dbnc_s2e != NULL) { delete this->_dbnc_s2e; }
    // if(this->_dbnc_s2c != NULL) { delete this->_dbnc_s2c; }

}

void X22_Climb::Periodic(bool Stage1_Ext, bool Stage1_Claw, bool Stage2_Ext, bool Stage2_Claw)
{
#ifndef CLIMB_INPUTS_AS_SEQUENCE

    // Parse input signals for rising edge changes
    if((this->rTrig_s1e != NULL) && (this->rTrig_s1c != NULL))  //&& (this->rTrig_s2e != NULL) && (this->rTrig_s2c != NULL))
    {
        if((this->_dbnc_s1e != NULL) && (this->_dbnc_s1e != NULL)) //&& (this->_dbnc_s2e != NULL) && (this->_dbnc_s2c != NULL))
        {
            this->rTrig_s1e->Check(this->_dbnc_s1e->Calculate(Stage1_Ext));
            this->rTrig_s1c->Check(this->_dbnc_s1c->Calculate(Stage1_Claw));
            // this->rTrig_s2e->Check(this->_dbnc_s2e->Calculate(Stage2_Ext));
            // this->rTrig_s2c->Check(this->_dbnc_s2c->Calculate(Stage2_Claw));
        }
        else
        {
            // Debouncer's are not instantiated, read button change directly.
            this->rTrig_s1e->Check(Stage1_Ext);
            this->rTrig_s1c->Check(Stage1_Claw);
            // this->rTrig_s2e->Check(Stage2_Ext);
            // this->rTrig_s2c->Check(Stage2_Claw);
        }

        this->stage1_ext_state  = (this->stage1_ext_state && (!this->rTrig_s1e->Q))     || (!this->stage1_ext_state && this->rTrig_s1e->Q);
        this->stage1_claw_state = (this->stage1_claw_state && (!this->rTrig_s1c->Q))    || (!this->stage1_claw_state && this->rTrig_s1c->Q);
        // this->stage2_ext_state  = (this->stage2_ext_state && (!this->rTrig_s2e->Q))     || (!this->stage2_ext_state && this->rTrig_s2e->Q);
        // this->stage2_claw_state = (this->stage2_claw_state && (!this->rTrig_s2c->Q))    || (!this->stage2_claw_state && this->rTrig_s2c->Q);
    }
    else
    {
        if((this->_dbnc_s1e != NULL) && (this->_dbnc_s1e != NULL)) // && (this->_dbnc_s2e != NULL) && (this->_dbnc_s2c != NULL))
        {
            bool s1e, s1c, s2e, s2c;

            s1e = this->_dbnc_s1e->Calculate(Stage1_Ext);
            s1c = this->_dbnc_s1c->Calculate(Stage1_Claw);
            // s2e = this->_dbnc_s2e->Calculate(Stage2_Ext);
            // s2c = this->_dbnc_s2c->Calculate(Stage2_Claw);

            this->stage1_ext_state = (this->stage1_ext_state && !s1e) || (!this->stage1_ext_state && s1e);
            this->stage1_claw_state = (this->stage1_claw_state && !s1c) || (!this->stage1_claw_state && s1c);
            // this->stage2_ext_state = (this->stage2_ext_state && !s2e) || (!this->stage2_ext_state && s2e);
            // this->stage2_claw_state = (this->stage2_claw_state && !s2c) || (!this->stage2_claw_state && s2c);
        }
        else
        {
            // Debouncer's are not instantiated, read button change directly.
            // Set the states based on the input parameters directly. This may be buggy and cause rapid toggling of the solenoids
            // since the inputs are state-dependent and not edge dependent
            this->stage1_ext_state = (this->stage1_ext_state && !Stage1_Ext) || (!this->stage1_ext_state && Stage1_Ext);
            this->stage1_claw_state = (this->stage1_claw_state && !Stage1_Claw) || (!this->stage1_claw_state && Stage1_Claw);
            // this->stage2_ext_state = (this->stage2_ext_state && !Stage2_Ext) || (!this->stage2_ext_state && Stage2_Ext);
            // this->stage2_claw_state = (this->stage2_claw_state && !Stage2_Claw) || (!this->stage2_claw_state && Stage2_Claw);
        }
    }

    if(this->_stage1_ext != NULL) { this->_stage1_ext->Set(this->stage1_ext_state); }
    if(this->_stage1_grab != NULL) { this->_stage1_grab->Set(this->stage1_claw_state); }
    // if(this->_stage2_ext != NULL) { this->_stage2_ext->Set(this->stage2_ext_state); }
    // if(this->_stage2_grab != NULL) { this->_stage2_grab->Set(this->stage2_claw_state); }

#else
    if(((_state + 1) == _nextstate) || (( _state - 1 ) == _nextstate)) {_prevstate = _state; _state = _nextstate; }
    else{_prevstate = _nextstate = _state;}

    /*
    * Climb squence: TODO: FIX THE NUMBERS IN PORTS CODE
    1) Stage 1 realease & Stage 2 realease
    2) Stage 1 extend
    3) Stage 1 grab
    4) Stage 1 retract
    5) Stage 2 extend
    6) Stage 2 grab
    7) Stage 1 release
    If stage 2 Adjustments is present -
    8) Stage 2 retract
    9) Stage 1 grab
    10) Stage 2 release
    11) Adj Stage 2
    12) Stage 2 extend
    13) Stage 2 grab
    14) Stage 1 release
    */

   switch(_state)
   {
       case 0: //Idle
       //TODO: Set default state of soleniods
            break;

       case 1: //Stage 1 realease & stage 2 realease
            this->_stage1_ext->Set(true);
            this->_stage2_grab->Set(true);
                break;

       case 2: //State 1 extend
            this->_stage1_grab->Set(true);
                break;

       case 3:  //Stage 1 grab
            this->_stage1_grab->Set(false);
                break;

       case 4: //Stage 1 retract
            this->_stage1_ext->Set(false);
                break;

       case 5: //Stage 2 extend
            this->_stage2_ext->Set(true);
                break;

       case 6: // stage 2 grab
            this->_stage2_grab->Set(false);
                break;

        case 7: //Stage 1 release
            this->_stage2_ext->Set(false);
                break;

    //   case 7: //Stage 2 retract

    //    case 8: //stage 1 grab

    //    case 9: //Stage 2 release

    //    case 10: //Adj Stage 2

    //    case 11 //Stage  2 extend
   }
#endif

}

void X22_Climb::SetSequence(int val)
{
    this->_nextstate = val;
}

