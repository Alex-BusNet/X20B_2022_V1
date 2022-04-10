#ifndef X22_INTAKE_H
#define X22_INTAKE_H

#include "FRC3484_Lib/components/SC_Limelight.h"
#include "FRC3484_Lib/components/SC_ColorSensor.h"
#include "FRC3484_Lib/components/SC_PID.h"

#include "FRC3484_Lib/utils/SC_Datatypes.h"
#include "FRC3484_Lib/utils/SC_Functions.h"
#include "FRC3484_Lib/utils/SC_Shuffleboard.h"

#include "ctre/phoenix/motorcontrol/can/TalonFX.h"
#include "ctre/phoenix/motorcontrol/can/VictorSPX.h"

#include "frc/Solenoid.h"
#include "frc/DigitalInput.h"
#include "frc/filter/Debouncer.h"

#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableValue.h"


class X22_Intake
{
public:
	X22_Intake(int IntakeID, int FeedID_Master, int FeedID_Slave, int LoaderRS_Ch,
				SC::SC_Solenoid Sol, int FeederSw_Ch, 
				frc::I2C::Port ColorSenPort);
	~X22_Intake();

	void Collect(bool Intake, bool ForceFeed, bool ForceEject);

	/**
	 * @brief Returns true if cargo is in the loader. 
	 */
	bool IsCargoLoaded();
	
	/**
	 * @brief Returns true if cargo is present in the feeder. This status will stay on for
	 *        a short period after the switch is released.
	 */
	bool IsCargoStored();

	/**
	 * @brief Returns true if the loader is down. Use to avoid running the feeder while launching cargo.
	 */
	bool IsLoaderDown();

private:

	void _initDashboard();
	void _updateDashboard();
	void _getColorDist();

	ctre::phoenix::motorcontrol::can::VictorSPX *Motor_Intake_Master;
	ctre::phoenix::motorcontrol::can::VictorSPX *Motor_Feed_Master;
	ctre::phoenix::motorcontrol::can::VictorSPX *Motor_Feed_Slave;
	frc::Solenoid *Sol_1;

	frc::DigitalInput *_di_feeder_sw, *_di_loader_rsw;
	SC::SC_ColorSensor *_sen_loader;

	nt::NetworkTableInstance _nt_inst;
	std::shared_ptr<nt::NetworkTable> _nt_table;

	SC::SC_ABFilter<double> *_filter_color_sen;
	double _color_dist_pvf;

	// Debouncers
	frc::Debouncer *_dbnc_re_feed_sw;       // Debounce the activation of the `cargo stored` switch
	frc::Debouncer *_dbnc_rf_intake;        // Debounce driver's `Intake` command on both the rising edge (R) and falling edge (F)
	frc::Debouncer *_dbnc_re_forceFeed;     // Debounce driver's `Force Feed` command on the rising edge (RE)
	frc::Debouncer *_dbnc_re_feedEject;     // Debounce driver's `Feed Eject` command on the rising edge (RE)
	frc::Debouncer *_dbnc_re_loaderDown;    // Debounce the loader down switch.
	frc::Debouncer *_dbnc_re_loaded;        // Dounce cargo loaded signal

	// Delay timers
	/* Debouncers can double as a delay timer for triggering an action after a specified time after a signal changes */
	frc::Debouncer *_dly_re_intake_on;  // Delay intake motors on after the rising edge (RE) of the driver's `Intake` command
	frc::Debouncer *_dly_fe_feed_off;   // Delay feed motors off after the falling edge (FE) of the switch un-depressing
	frc::Debouncer *_dly_fe_loaded;		// Delay `cargo loaded` signal off after the falling edge (FE) of the sensor signal
};


#endif // X22_INTAKE_H
