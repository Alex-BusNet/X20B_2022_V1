#ifndef SC_COLORSENSOR_H
#define SC_COLORSENSOR_H

#include "frc/util/Color.h"
#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"

namespace SC //it is in yours Porthalomule
{
    class SC_ColorSensor
    {
    public:
        SC_ColorSensor(frc::I2C::Port I2CPort);
        ~SC_ColorSensor();

        void AddColor(frc::Color Color);

        /**
         * @brief Returns the detected color. Returns `kBlack` if the sensor is null.
         */
        frc::Color GetColor();

        /**
         * @brief Returns the closest detected color. Returns `kBlack` if the sensor is null.
         */
        frc::Color MatchClosest(frc::Color ColorToMatch, double& Confidence);

        /**
         * @brief Returns the distance from the sensor on the range of [0, 2047]. Large values are closer to the sensor. Returns 0.0 if sensor is null.
         */
        double GetDistance();

    private:
        rev::ColorSensorV3 *sensor;
        rev::ColorMatch *colorMatcher;
    };

}

#endif // SC_COLORSENSOR_H