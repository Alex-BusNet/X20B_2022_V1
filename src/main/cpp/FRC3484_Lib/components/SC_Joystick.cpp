#include "FRC3484_Lib/components/SC_Joystick.h"

#include <hal/FRCUsageReporting.h>
#include <cmath>
#include <wpi/numbers>

using namespace SC;
using namespace frc;

SC_Joystick::SC_Joystick(int port) : GenericHID(port)
{
	this->_axes[_Axis::kX] = LE3D_X;
	this->_axes[_Axis::kY] = LE3D_Y;
	this->_axes[_Axis::kZ] = LE3D_Z;
	this->_axes[_Axis::kTwist] = LE3D_Z;
	this->_axes[_Axis::kThrottle] = LE3D_THROTTLE;

	HAL_Report(HALUsageReporting::kResourceType_Joystick, port + 1);
}

double SC_Joystick::GetXAxis()
{
	return this->GetRawAxis(LE3D_X);
}

double SC_Joystick::GetYAxis()
{
	return this->GetRawAxis(LE3D_Y);
}

double SC_Joystick::GetZAxis()
{
	return this->GetRawAxis(LE3D_Z);
}

double SC_Joystick::GetTwistAxis()
{
	return this->GetRawAxis(LE3D_Z);
}

double SC_Joystick::GetThrottleAxis()
{
	return this->GetRawAxis(LE3D_THROTTLE);
}

int SC_Joystick::GetButton(int button)
{
	return this->GetRawButton(button);
}