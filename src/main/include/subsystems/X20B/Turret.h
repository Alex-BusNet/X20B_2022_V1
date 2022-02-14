#ifndef TURRET_H
#define TURRET_H

#include "ctre/phoenix/motorcontrol/can/VictorSPX.h"

#include "FRC3484_Lib/subsystems/SC_MotorSS.h"
#include "FRC3484_Lib/components/SC_PID.h"
#include "units/angle.h"
#include "units/time.h"

class Turret : public SC::SC_MotorSS
{
public:
    Turret(int channel);
    Turret(int channel, SC::SC_PIDConstants PIDc);

    /**
     * @brief   Deconstructor. Cleans up any allocated memory
     *          from object before disposing.
     */
    ~Turret();

    /**
     * @brief   
     */
    void Periodic() override;
    
    /**
     * @brief   Prepares the turret to operate. Actual
     *          operation of turret is handled in Periodic().
     */
    void Run();

    /**
     * @brief   Safely stops the turret. This disables
     *          operation of the turret in Perodic()
     */
    void Stop();

    void SetTargetPositionWindow(units::angle::degree_t window);
    void SetTargetPositionTime(units::time::second_t time);

    /**
     * @brief   Sets the target angle of the turret.
     * 
     */
    void SetPositionSP(units::angle::degree_t angle);

    /**
     * @brief   Returns TRUE if the turret is within tolerance
     *          of the target position.
     */
    bool IsTurretInPosition();

private:
    units::angle::degree_t theta; // Rotational angle for launching
    units::angle::degree_t pos; // Actual position of turret
    units::angle::degree_t tol; // In Position acceptance window

    units::time::second_t holdTime; // Time position must be within tolerance before "InPosition" is TRUE

    bool trackerEn; // Vision tracking enabled

    ctre::phoenix::motorcontrol::can::VictorSPX *TurretMotor;

    SC::SC_PID *turretPID;

};
#endif // TURRET_H