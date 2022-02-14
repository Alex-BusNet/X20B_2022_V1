#ifndef LAUNCHER_H
#define LAUNCHER_H

/*
 *   Desc: Launcher Subsystem
 * Author: Port
 *   Date: 7/19/2020
 */
#include "FRC3484_Lib/subsystems/SC_MotorSS.h"
#include "FRC3484_Lib/components/SC_PID.h"
#include "FRC3484_Lib/utils/SC_Functions.h"

#include "ctre/phoenix/motorcontrol/can/TalonFX.h"

#include "units/velocity.h"
#include "units/angular_velocity.h"
#include "units/length.h"
#include "units/time.h"

#include "Constants.h"

#include <ratio>


using namespace ctre::phoenix::motorcontrol::can;
using namespace ctre::phoenix::motorcontrol;

using namespace units::dimensionless;

class Launcher : public SC::SC_MotorSS
{
public:
    Launcher(int channel, SC::SC_PIDConstants PIDc);
    Launcher(int channel, SC::SC_PIDConstants PIDc, std::ratio<1> GR);

    /**
     * @brief   Deconstructor. Cleans up any allocated memory
     *          from object before disposing.
     */
    ~Launcher();

    /**
     * @brief   Reads the wheel speed of the launcher. This
     *          also handles operation of the PID controller 
     *          if the Run() function was called.
     */
    void Periodic() override;

    /**
     * @brief   Prepares the launcher to operate. Actual 
     *          operation of launcher is handled in 
     *          Periodic().
     */
    void Run();

    /**
     * @brief   Safely stops the launcher. This disables
     *          operation of launcher in Periodic().
     */
    void Stop();

    /**
     * @brief   Change the launcher velocity SP. Changing
     *          this value while the loop is running will
     *          cause the setpoint to step down to the new SP.
     * 
     * @param   value   The new launch velocity SP.
     */
    void SetVelocitySP(units::revolutions_per_minute_t value);

    /**
     * @brief   Set the max velocity output of the launcher.
     *          The setpoint cannot be larger than the configured
     *          max velocity, and is capped to the max velocity
     *          inside the function.
     * 
     * @param   value   The max velocity of the launcher.
     */
    void SetMaxVelocity(units::revolutions_per_minute_t value);

    /**
     * @brief   Used to set the tolerance band to determine if
     *          the launcher is in position
     * 
     * @param   tol     Size of window to be considered "in position"
     */
    void SetTolerance(units::revolutions_per_minute_t tol);

    /**
     * @brief   Used to prevent the CV from changing too rapidly.
     * 
     * @param   value     CV rate-of-change limit
     */
    void SetRateLimit(double value);

    /**
     * @brief   Ramp rate is used to increment the SP fed to the
     *          PID controller upto the desired SP. This helps
     *          prevent damage to the subsystem when starting from
     *          rest.
     * 
     * @param   step    Amount to increase setpoint fed to PID controller
     *                  after each interval has elapsed.
     */
    void SetRampRate(units::revolutions_per_minute_t step);

    /**
     * @brief   Overloaded function definition of SetRampRate(feet_per_second_t).
     *          
     * @param   interval    Time, in milliseconds, to wait before
     *                      increasing the setpoint.
     * @param   step        Amount to increase setpoint fed to PID controller
     *                      after each interval has elapsed.
     */
    void SetRampRate(units::revolutions_per_minute_t step, units::millisecond_t interval);

    /**
     * @brief   Returns the current wheel speed in ft/sec as a double.
     */
    double GetMeasurement();

    /**
     * @brief   Returns true if the current wheel speed is within the 
     *          configured tolerance of the setpoint.
     */
    bool InPosition();

private:
    void UseOutput(double output);

    units::revolutions_per_minute_t velocitySP, tolerance, rampRate, sp_feed;
    units::revolutions_per_minute_t maxVel, wheelSpeed;
    
    units::foot_t wheelRadius;

    double cv, cv_last;
    double rateLim;
    double gearRatio;
    
    units::millisecond_t rampInterval, timeElapsed, lastTime;
    
    bool enabled;
    
    int sensorCounts;

    SC::SC_PID  *controller;

    TalonFX *launchMotor;
};


#endif // LAUNCHER_H