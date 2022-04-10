#include "subsystems/X22/X22_Intake.h"

#include "subsystems/X22/X22_Constants.h"
#include "Globals.h"

using namespace frc;
using namespace SC;
using namespace nt;
using namespace units::length;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

X22_Intake::X22_Intake(int IntakeID, int FeedID_Master, int FeedID_Slave, int LoaderRS_Ch,
	SC_Solenoid Sol, int FeederSw_Ch,
	frc::I2C::Port ColorSenPort)
{

	Motor_Intake_Master = new VictorSPX(IntakeID);
	Motor_Intake_Master->SetNeutralMode(NeutralMode::Coast);
	Motor_Intake_Master->ConfigOpenloopRamp(C_INTAKE_RAMP_TIME);
	Motor_Intake_Master->SetInverted(true);

	Motor_Feed_Master = new VictorSPX(FeedID_Master);
	Motor_Feed_Master->SetNeutralMode(NeutralMode::Brake);
	Motor_Feed_Master->SetInverted(true);

	Motor_Feed_Slave = new VictorSPX(FeedID_Slave);
	Motor_Feed_Slave->SetNeutralMode(NeutralMode::Brake);
	Motor_Feed_Slave->Follow(*Motor_Feed_Master);

	Sol_1 = new Solenoid(Sol.CtrlID, Sol.CtrlType, Sol.Channel);
	Sol_1->Set(false);

	_di_feeder_sw = new DigitalInput(FeederSw_Ch);
	_di_loader_rsw = new DigitalInput(LoaderRS_Ch);

	this->_sen_loader = new SC_ColorSensor(ColorSenPort);

	this->_filter_color_sen = new SC_ABFilter<double>(0.200_s, 0.020_s);

	this->_dbnc_rf_intake = new Debouncer(C_INTAKE_BTN_DBNC_TIME, Debouncer::kBoth); // Prevent accidental deployment or release of intake
	this->_dbnc_re_feed_sw = new Debouncer(C_CARGOSTORED_DBNC_TIME, Debouncer::kRising);
	this->_dbnc_re_loaderDown = new Debouncer(C_LOADER_SW_DBNC_TIME, Debouncer::kRising);
	this->_dbnc_re_forceFeed = new Debouncer(C_MANUFEED_DBNC_TIME, Debouncer::kRising);
	this->_dbnc_re_feedEject = new Debouncer(C_FEEDEJECT_DBNC_TIME, Debouncer::kRising);
	this->_dbnc_re_loaded = new Debouncer(C_CARGOLOADED_DBNC_TIME, Debouncer::kRising);

	this->_dly_re_intake_on = new Debouncer(C_INTAKEMOTOR_DELAY_TIME, Debouncer::kRising); // Delay intake motors on until the intake is extended partially
	this->_dly_fe_feed_off = new Debouncer(C_AUTOFEED_OFF_DELAY_TIME, Debouncer::kFalling);
	this->_dly_fe_loaded = new Debouncer(C_LOADED_OFF_DELAY_TIME, Debouncer::kFalling);

	this->_initDashboard();
}

X22_Intake::~X22_Intake()
{
	if(Motor_Intake_Master != NULL) { delete Motor_Intake_Master; }
	if(Motor_Feed_Master != NULL) {delete Motor_Feed_Master; }
	if(Motor_Feed_Slave != NULL) {delete Motor_Feed_Slave; }

	if(Sol_1 != NULL) { delete Sol_1; }
	if(_di_feeder_sw != NULL) { delete _di_feeder_sw; }
	if(_di_loader_rsw != NULL) { delete _di_loader_rsw; }
	if(_sen_loader != NULL) { delete _sen_loader; }

	if(this->_dbnc_re_feed_sw != NULL) { delete this->_dbnc_re_feed_sw; }
	if(this->_dbnc_re_feedEject != NULL) { delete this->_dbnc_re_feedEject; }
	if(this->_dbnc_re_forceFeed != NULL) { delete this->_dbnc_re_forceFeed; }
	if(this->_dbnc_re_loaded != NULL) { delete this->_dbnc_re_loaded; }
	if(this->_dbnc_re_loaderDown != NULL) { delete this->_dbnc_re_loaderDown; }
	if(this->_dbnc_rf_intake != NULL) { delete this->_dbnc_rf_intake; }

	if(this->_dly_fe_feed_off != NULL) { delete this->_dly_fe_feed_off; }
	if(this->_dly_re_intake_on != NULL) { delete this->_dly_re_intake_on; }
	if(this->_dly_fe_loaded != NULL) { delete this->_dly_fe_loaded; }

	if(this->_filter_color_sen != NULL) { delete this->_filter_color_sen; }
}

void X22_Intake::Collect(bool Run, bool ForceFeed, bool ForceEject)
{
	bool intakeOut, intakeForward, intakeReverse, feederForward, feederReverse;
	double intakeSpeed, feederSpeed;

	bool cargoStored = IsCargoStored(); // Save the state of the cargo switch

	// Deploy the intake
	if(this->_dbnc_rf_intake != NULL) { intakeOut = this->_dbnc_rf_intake->Calculate(Run); }
	else { intakeOut = Run; }

	// Turn on the intake motor
	if(this->_dly_re_intake_on != NULL) { intakeForward = this->_dly_re_intake_on->Calculate(intakeOut || ForceFeed); }
	else { intakeForward = intakeOut || ForceFeed; }

	// Run the feed motor forward (towards loader)
	if(this->_dbnc_re_forceFeed != NULL) 
	{ 
		feederForward = (intakeForward && (!cargoStored || (!IsCargoLoaded() && cargoStored))) // Auto-feed
						|| this->_dbnc_re_forceFeed->Calculate(ForceFeed); // Manual feed
	}
	else { feederForward = intakeForward || ForceFeed; }

	// Run the feed motor reverse (away from loader)
	if(this->_dbnc_re_feedEject != NULL) { intakeReverse = feederReverse = this->_dbnc_re_feedEject->Calculate(ForceEject); }
	else { intakeReverse = feederReverse = ForceEject; }

	// Set the motor speeds
	if(intakeForward) { intakeSpeed = C_INTAKE_DRIVE_SPEED; }
	else if(intakeReverse) { intakeSpeed = -C_INTAKE_DRIVE_SPEED; }
	else { intakeSpeed = 0.0; }

	if(feederForward) { feederSpeed = C_FEED_DRIVE_SPEED; }
	else if(feederReverse) { feederSpeed = -C_FEED_DRIVE_SPEED; }
	else { feederSpeed = 0.0; }

	// Apply outputs
	Sol_1->Set(intakeOut);
	Motor_Intake_Master->Set(ControlMode::PercentOutput, intakeSpeed);
	Motor_Feed_Master->Set(ControlMode::PercentOutput, feederSpeed);

	// Update dashboard statuses
	this->_updateDashboard();
}

bool X22_Intake::IsCargoLoaded()
{
	bool switchState; // Save the state of the input at the time of this function call (pre-debounced)
	static bool switchOut; // Remember the state of switch out within the context of this function

	if(this->_sen_loader != NULL) 
	{
		if(this->_filter_color_sen != NULL)	{ _color_dist_pvf = this->_filter_color_sen->Filter(this->_sen_loader->GetDistance()); }
		else { _color_dist_pvf = this->_sen_loader->GetDistance(); }
	}
	else { _color_dist_pvf = 2047.0; } // Color sensor values are larger closer to the sensor.
	
	// Set the current state of the 'switch'
	switchState = _color_dist_pvf > C_CARGO_LOADED_THRESH;

	// Debounce the status.
	if((this->_dbnc_re_loaded != NULL) && (this->_dly_fe_loaded != NULL)) 
	{ 
		switchOut = this->_dbnc_re_loaded->Calculate(switchState) 					// Turn on the status after it is true for X seconds 
					|| (switchOut && this->_dly_fe_loaded->Calculate(switchState)); // status latch, Turn off the status after the switch is false for Y seconds
	}
	else { switchOut = switchState; }

	return switchOut;
}

bool X22_Intake::IsCargoStored()
{
	bool switchState = false; // Save the state of the input at the time of this function call (pre-debounced)
	static bool switchOut = false; // Remember the state of switch out within the context of this function

	if(this->_di_feeder_sw != NULL)
	{
		switchState = this->_di_feeder_sw->Get();
		if((this->_dbnc_re_feed_sw != NULL) && (this->_dly_fe_feed_off != NULL))
		{
			switchOut = this->_dbnc_re_feed_sw->Calculate(switchState); // Turn on the signal
						//|| (switchOut && this->_dly_fe_feed_off->Calculate(switchState)); // Turn off the status X second
		}
		else { switchOut = switchState; }
	}

	return switchOut;
}

bool X22_Intake::IsLoaderDown()
{
	if(this->_di_loader_rsw != NULL)
	{
		if(this->_dbnc_re_loaderDown != NULL) { return this->_dbnc_re_loaderDown->Calculate(this->_di_loader_rsw->Get()); }
		else { return this->_di_loader_rsw->Get(); }
	}
	else { return true; } // If the switch is NULL, assume the loader is always down
}

void X22_Intake::_initDashboard()
{
	this->_nt_inst = NetworkTableInstance::GetDefault();
	this->_nt_table = this->_nt_inst.GetTable("X22");

	this->_nt_table->PutNumber("Color Sen Dist", 0.0);
	this->_nt_table->PutBoolean("Cargo Stored", false);
	this->_nt_table->PutBoolean("Loader Down", false);
	this->_nt_table->PutBoolean("Cargo Loaded", false);
}

void X22_Intake::_updateDashboard()
{
	this->_nt_table->PutNumber("Color Sen Dist", this->_color_dist_pvf);
	this->_nt_table->PutBoolean("Cargo Stored", this->IsCargoStored());
	this->_nt_table->PutBoolean("Loader Down", this->IsLoaderDown());
	this->_nt_table->PutBoolean("Cargo Loaded", this->IsCargoLoaded());
}
