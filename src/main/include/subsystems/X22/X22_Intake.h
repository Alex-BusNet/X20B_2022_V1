#ifndef X22_INTAKE_H
#define X22_INTAKE_H

#include "ctre/phoenix/motorcontrol/can/VictorSPX.h"

#include "FRC3484_Lib/utils/SC_Datatypes.h"

#include "frc/Solenoid.h"
#include "frc/DigitalInput.h"

class X22_Intake
{
public:
    X22_Intake(int MotorID, SC::SC_Solenoid Sol, int ProxSen1_Ch, int ProxSen2_Ch);
    ~X22_Intake();


private:
    ctre::phoenix::motorcontrol::can::VictorSPX Motor_Master;
    frc::Solenoid Sol_1;

    frc::DigitalInput DI_Prox;

};


#endif