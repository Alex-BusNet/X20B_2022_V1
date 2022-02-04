#ifndef SC_PNEUMATICSS_H
#define SC_PNEUMATICSS_H

#include "frc2/command/subsystembase.h"

namespace SC
{
    class SC_PneumaticSS : public frc2::SubsystemBase
    {
    public:
        SC_PneumaticSS();

        void Extend();
        void Retract();

    private:
        
    };
}

#endif