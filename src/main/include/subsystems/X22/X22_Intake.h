#ifndef X22_INTAKE_H
#define X22_INTAKE_H

#include "ctre/phoenix/motorcontrol/can/VictorSPX.h"

#include "FRC3484_Lib/utils/SC_Datatypes.h"
#include "FRC3484_Lib/components/SC_ColorSensor.h"

#include "frc/Solenoid.h"
#include "frc/DigitalInput.h"

class X22_Intake
{
public:
    X22_Intake(int IntakeID, int FeedID_Master, int FeedID_Slave,
                SC::SC_Solenoid Sol, int ProxSen_Ch, 
				frc::I2C::Port ColorSenPort);
    ~X22_Intake();

    void Collect(bool Run);

    bool IsCargoLoaded();

private:
    ctre::phoenix::motorcontrol::can::VictorSPX *Motor_Intake_Master;
    ctre::phoenix::motorcontrol::can::VictorSPX *Motor_Feed_Master;
    ctre::phoenix::motorcontrol::can::VictorSPX *Motor_Feed_Slave;
    frc::Solenoid *Sol_1;

    frc::DigitalInput *DI_Prox;
	SC::SC_ColorSensor *_sen_loader;

};


#endif // X22_INTAKE_H