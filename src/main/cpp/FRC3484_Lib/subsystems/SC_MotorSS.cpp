#include "FRC3484_Lib/subsystems/SC_MotorSS.h"
using namespace SC;

SC_MotorSS::SC_MotorSS() 
    : frc2::Subsystem()
{
    // c'tor
    this->name = "SC_MotorSS";
}

SC_MotorSS::SC_MotorSS(std::string SSName) 
    : frc2::Subsystem()
{
    // c'tor
    this->name = SSName;
}

SC_MotorSS::~SC_MotorSS()
{
    // Dec'tor
}

std::string SC_MotorSS::GetName()
{
    return this->name;
}