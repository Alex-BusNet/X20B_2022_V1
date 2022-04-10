#ifndef SC_JOYSTICK_H
#define SC_JOYSTICK_H

#include <frc/GenericHID.h>
#include "FRC3484_Lib/utils/SC_ControllerMaps.h"
#include <array>

namespace SC
{

class SC_Joystick : public frc::GenericHID
{
public:
	explicit SC_Joystick(int port);
	~SC_Joystick() override = default;

	double GetXAxis();
	double GetYAxis();
	double GetZAxis();
	double GetTwistAxis();
	double GetThrottleAxis();

	int GetHatAxis(int axis);

	int GetButton(int button);

	double GetMagnitude() const;

	double GetDirectionRadians() const;

	double GetDirectionDegrees() const;

private:
	enum _Axis {kX, kY, kZ, kTwist, kThrottle, kNumAxes };

	std::array<int, _Axis::kNumAxes> _axes;
};

}

#endif // SC_JOYSTICK_H