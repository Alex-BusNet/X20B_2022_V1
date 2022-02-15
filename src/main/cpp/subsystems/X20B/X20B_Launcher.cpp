#include "subsystems/X20B/Launcher.h"
#include "FRC3484_Lib/utils/SC_Functions.h"
#include "FRC3484_Lib/utils/SC_Constants.h"

using namespace SC;
using namespace units;

Launcher::Launcher(int channel, SC_PIDConstants PIDc) : SC_MotorSS("Launcher")
{
    launchMotor = new TalonFX(channel);

    launchMotor->ConfigSelectedFeedbackSensor(TalonFXFeedbackDevice::IntegratedSensor, 0, 0);
    launchMotor->SetNeutralMode(NeutralMode::Coast);

    this->controller = new SC_PID(PIDc);

    this->rateLim = 0.2;
    this->rampRate = 1_rpm;
    this->rampInterval = 0.5_s;
    this->wheelRadius = 1_in;
    this->maxVel = 10_rpm;
    this->gearRatio = 1.0;
}

Launcher::Launcher(int channel, SC_PIDConstants PIDc, std::ratio<1> GR) 
    : Launcher(channel, PIDc)
{
    this->gearRatio = ((double)GR.num) / ((double)GR.den); 
}

Launcher::~Launcher()
{
    if(this->controller != nullptr)
        delete this->controller;
    
    if(this->launchMotor != nullptr)
        delete this->launchMotor;
}

void Launcher::Run()
{
    this->controller->SetSP(F_Percent(velocitySP, maxVel));
    this->controller->Enable();
    this->enabled = true;

    timeElapsed = 0_ms;
    lastTime = 0_ms;
}

void Launcher::Periodic()
{
    // Fetch the current speed of the launch wheel
    if (launchMotor != nullptr)
    {
        if(gearRatio != 0)
        {
            sensorCounts = launchMotor->GetSelectedSensorVelocity(0);
            // Calculate RPM from sensor counts.
            wheelSpeed =  (sensorCounts / (4096.0 / gearRatio)) * 600.0_rpm;
            // Convert RPM to ft/sec - the units data type should (may) handle the type conversions.
            //wheelSpeed = make_unit<feet_per_second_t>(wheelSpeedrpm.to<double>() (1 / 60.0) * wheelRadius.to<double>()); //((2 * PI_VAL) / 1_rev) * wheelRadius * (1_min / 60_s); 
            //wheelSpeed = (units::convert<rpm, rad_per_s>(wheelSpeedrpm) / SC::C_REVOLUTION_RADS) * wheelRadius; 
        }
        else
        {
            wheelSpeed = 0_rpm;
        }
    }
    else
    {
        wheelSpeed = 0_rpm;
    }

    // Run the PID Controller.
    if(enabled)
    {
        timeElapsed += SC::C_CYCLE_TIME;

        // Ramp the setpoint up to the target speed.
        if(((timeElapsed - lastTime) > rampInterval)) 
        {
            lastTime = timeElapsed;
            
            if(!InPosition())
            {
                if(wheelSpeed < velocitySP) // Step up to SP
                    sp_feed = F_Limit(-maxVel, velocitySP, sp_feed + rampRate);
                else // Step down to SP
                    sp_feed = F_Limit(velocitySP, maxVel, sp_feed - rampRate);
            }
        }

        if(controller != nullptr)
            UseOutput(controller->Calculate(GetMeasurement(), F_Percent(sp_feed.value(), maxVel.value())));
        else
            UseOutput(0.0);
    }
}

void Launcher::Stop()
{
    UseOutput(0.0);
    this->controller->Disable();
    this->enabled = false;
}

void Launcher::SetVelocitySP(revolutions_per_minute_t value)
{
    this->velocitySP = F_Limit(0_rpm, this->maxVel, value);
}

void Launcher::SetMaxVelocity(revolutions_per_minute_t value)
{
    this->maxVel = value;
}

void Launcher::SetTolerance(revolutions_per_minute_t tol)
{
    this->tolerance = tol;
}

void Launcher::SetRateLimit(double value)
{
    this->rateLim = value;
}

void Launcher::SetRampRate(revolutions_per_minute_t step)
{
    this->rampRate = step;
}

void Launcher::SetRampRate(revolutions_per_minute_t step, millisecond_t interval)
{
    this->rampRate = step;
    this->rampInterval = interval;
}

void Launcher::UseOutput(double output)
{
    cv_last = cv;

    // Limit the CV to be between 0 and 100%
    cv = F_Limit(output, 0.0, 1.0);

    // Enforce CV rate limit
    cv = SC::F_Limit(cv_last - rateLim, cv_last + rateLim, cv);

    // Send the output to the motor
    if(launchMotor != nullptr)
    {
        launchMotor->Set(ControlMode::PercentOutput, cv);
    }
}

bool Launcher::InPosition()
{
    return ((velocitySP + tolerance) > wheelSpeed) 
        && ((velocitySP - tolerance) < wheelSpeed);
}

double Launcher::GetMeasurement()
{
    return wheelSpeed.to<double>();
}