#include "FRC3484_Lib/Components/SC_ColorSensor.h"

using namespace frc;
using namespace SC;
using namespace rev;

SC_ColorSensor::SC_ColorSensor(I2C::Port I2CPort)
{
	sensor = new ColorSensorV3(I2CPort);
	colorMatcher = new ColorMatch();
}

SC_ColorSensor::~SC_ColorSensor()
{
	if(sensor != NULL) { delete sensor; }
	if(colorMatcher != NULL) { delete sensor; }
}

void SC_ColorSensor::AddColor(Color Color)
{
	if(this->colorMatcher != NULL) { this->colorMatcher->AddColorMatch(Color); }
}

Color SC_ColorSensor::GetColor()
{
	if(this->sensor != NULL) { return this->sensor->GetColor(); }
	else { return Color::kBlack; }
}

Color SC_ColorSensor::MatchClosest(Color ColorToMatch, double& Confidence)
{
	if(this->colorMatcher != NULL) { return this->colorMatcher->MatchClosestColor(ColorToMatch, Confidence); }
	else { return Color::kBlack; }
}

double SC_ColorSensor::GetDistance()
{
	if(this->sensor != NULL) { return this->sensor->GetProximity(); }
	else {return 0.0; }
}