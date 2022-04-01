#ifndef SC_SUBSYSTEMBASE_H
#define SC_SUBSYSTEMBASE_H

#include <frc2/command/Subsystem.h>

/*
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/velocity.h"
#include "units/length.h"
#include "units/time.h"

#include "FRC3484_Lib/utils/SC_Constants.h"
#include "FRC3484_Lib/utils/SC_Datatypes.h"
#include "FRC3484_Lib/utils/SC_Functions.h"
*/

#include <string.h>

namespace SC
{
    class SC_MotorSS : public frc2::Subsystem
    {
    public:
        SC_MotorSS();
        SC_MotorSS(std::string SSName);

        ~SC_MotorSS();

        void Periodic() { };

        virtual void Run() { };
        virtual void Stop() { };

        std::string GetName();

    protected:
        virtual void _init_NT_Data() { };

    private:
        std::string name;

    };
}
#endif