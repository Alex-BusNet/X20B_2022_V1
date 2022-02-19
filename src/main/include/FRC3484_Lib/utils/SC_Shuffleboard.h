#ifndef SC_SHUFFLEBOARD_H
#define SC_SHUFFLEBOARD_H

#include "frc/shuffleboard/Shuffleboard.h"
#include "frc/shuffleboard/BuiltInWidgets.h"
#include "frc/shuffleboard/BuiltInLayouts.h"

#include "networktables/NetworkTableEntry.h"

#include "wpi/StringMap.h"

#include <cstring>

namespace SC
{
    // Forward declaration of shuffleboard objects
    class SC_SBInput;
    class SC_SBOutput;
    class SC_SBLayout;

/* ============================================================================== */

    /**
     * @brief Shuffleboard builder wrapper class
     */
    class SC_SBBuilder
    {
    public:
        SC_SBBuilder(std::string TabName);
        ~SC_SBBuilder();

        SC::SC_SBInput* AddBooleanInput();
        SC::SC_SBInput* AddNumericInput();
        SC::SC_SBInput* AddComboBoxInput();
        
        SC::SC_SBOutput* AddBooleanOutput();
        SC::SC_SBOutput* AddNumericOutput();

        SC::SC_SBLayout* CreateLayout();
        
    private:
        std::string _tabName;
    };

/* ============================================================================== */

    /**
     * @brief Shuffleboard input (Dashboard -> RoboRIO) object
     */
    class SC_SBInput
    {
    public:
        SC_SBInput();
        ~SC_SBInput();

    private:
        wpi::StringMap<std::shared_ptr<nt::Value>> _props;
    };

/* ============================================================================== */


    /**
     * @brief Shuffleboard output (RoboRIO -> Dashboard) object
     */
    class SC_SBOutput
    {
    public:
        SC_SBOutput();
        ~SC_SBOutput();

    private:
        wpi::StringMap<std::shared_ptr<nt::Value>> _props;
    };

/* ============================================================================== */

    /**
     * @brief Shuffleboard layout helper class.
     */
    class SC_SBLayout
    {
    public:
        SC_SBLayout();
        ~SC_SBLayout();

    private:

    };
}

#endif // SC_SHUFFLEBOARD_H