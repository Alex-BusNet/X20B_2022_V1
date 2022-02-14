#include "subsystems/X20B/Turret.h"

using namespace std;
using namespace units::angle;
using namespace units::time;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

using namespace SC;

Turret::Turret(int channel)
    : SC_MotorSS("Turret")
{
    TurretMotor = new VictorSPX(channel);
    this->turretPID = new SC_PID();
    this->theta = 0_deg;
}

Turret::Turret(int channel, SC_PIDConstants PIDc)
    : SC_MotorSS("Turret")
{
    TurretMotor = new VictorSPX(channel);
    this->TurretMotor->SetNeutralMode(NeutralMode::Brake);

    this->turretPID = new SC_PID(PIDc, SC_PID_AW_MODE::INTEGRAL_LIMIT); 
    
    this->theta = 0_deg;
}

Turret::~Turret()
{
    if(this->turretPID != nullptr)
    {
        delete this->turretPID;
    }

    if(this->TurretMotor != nullptr)
    {
        delete this->TurretMotor;
    }
}

void Turret::Periodic()
{
    if(trackerEn)
    {
        if(!this->turretPID->IsEnabled())
        {
            this->turretPID->Enable();
        }

        /// TODO: Add limelight and chameleon integration to turret  
    }
}

void Turret::Run()
{
    this->trackerEn = true;
}

void Turret::Stop()
{
    this->trackerEn = false;
}

bool Turret::IsTurretInPosition()
{
    return false;
}

void Turret::SetTargetPositionWindow(degree_t window)
{
    this->tol = window;
}

void Turret::SetTargetPositionTime(second_t time)
{
    this->holdTime = time;
}

void Turret::SetPositionSP(degree_t angle)
{
    this->theta = angle;
}