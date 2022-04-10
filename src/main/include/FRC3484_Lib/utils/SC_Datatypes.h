#ifndef SC_DATATYPES_H
#define SC_DATATYPES_H


//#include "TC3_Syntax.h"
#include <tuple>
#include <algorithm>
#include <stdexcept>

#include "frc/PneumaticsModuleType.h"
#include "frc/geometry/Translation2d.h"

#include "ctre/phoenix/motorcontrol/can/TalonFX.h"

#include "units/time.h"
#include "units/math.h"

#include "SC_Filters.h"

namespace SC
{
	template<class T>
	struct SC_Range 
	{ 
		T Val_min; 
		T Val_max; 

		SC_Range& operator=(SC_Range& rhs)
		{
			this->Val_max = std::max(rhs.Val_max, rhs.Val_min);
			this->Val_min = std::min(rhs.Val_max, rhs.Val_min);

			return *this;
		}

		SC_Range& operator()(T val1, T val2)
		{
			this->Val_max = std::max(val1, val2);
			this->Val_min = std::min(val1, val2);

			return *this;
		}

		SC_Range& operator=(std::tuple<T, T> rhs)
		{
			this->Val_max = std::max(std::get<0>(rhs), std::get<1>(rhs));
			this->Val_min = std::min(std::get<0>(rhs), std::get<1>(rhs));

			return *this;
		}
	};

	/*========================*/
	/*=== SC_PID Datatypes ===*/
	/*========================*/

	// PID loop Integral Anti-Windup calculation modes
	enum SC_PID_AW_MODE { OFF, /*
						BACK_CALCULATION, 
						CONDITIONAL_INTEGRAL, */
						INTEGRAL_LIMIT };

	typedef struct {double Kp; double Ki; double Kd; double Kf;} SC_PIDConstants;
	typedef struct {
		SC_PIDConstants PIDc; 
		units::time::second_t ScanTime;		// Update period of the PID loop
		SC_Range<double> R_SP;				// Setpoint bounding range
		SC_Range<double> R_PV;				// PV (sensor input) bounding range
		SC_Range<double> R_CV;				// CV (output) bounding range
		SC_Range<double> R_I;				// Integral bounding range
		SC_Range<double> R_D; 				// Derivative bouding range
		double ManRate; 					// Manual mode CV ramp rate
		SC_PID_AW_MODE AW_Mode = OFF;		// Integral Anti-windup mode (default = OFF)
	} SC_PIDConfig;

	typedef struct {
		double P;   // Current value of the P term 
		double I;   // Current value of the I term
		double D;   // Current value of the D term
		double E;   // Current error value
		double Elast; // Last error value
		SC_PID_AW_MODE AntiWindup_Mode; // Configured Anti-windup method
		double Kw;  // Anti-Windup Coeff value
		bool enabled; // Enabled status of PID Controller
		bool reversed; // Reversed status of PID Controller
		double SP; // Current setpoint
		double PV;
		double CV;
	} SC_PIDStatus;


	/*==========================================*/
	/*=== Pneumatic Configuration Structures ===*/
	/*==========================================*/
	typedef struct {int CtrlID; frc::PneumaticsModuleType CtrlType; int Channel;} SC_Solenoid;
	typedef struct {int CtrlID; frc::PneumaticsModuleType CtrlType; int Fwd_Channel; int Rev_Channel;} SC_DoubleSolenoid;

	/*===========================================*/
	/*=== Drivetrain Configuration Structures ===*/
	/*===========================================*/
	typedef struct { int CANId; ctre::phoenix::motorcontrol::TalonFXInvertType InvDir; } SC_Motor_OL; // Open-loop motor definition
	typedef struct { int CANId; ctre::phoenix::motorcontrol::TalonFXInvertType InvDir; double ScaleFactor; SC_PIDConfig PIDCfg; } SC_Motor_CL; // Closed-loop motor definition

	typedef struct { SC_Motor_CL DriveMotor; SC_Motor_CL RotMotor; frc::Translation2d ModuleLoc; int EncID; } SC_SwerveModuleConfig;
	typedef struct { SC_SwerveModuleConfig FL_cfg; SC_SwerveModuleConfig FR_cfg; SC_SwerveModuleConfig BL_cfg; SC_SwerveModuleConfig BR_cfg; } SC_SwerveConfig;

	typedef struct {double throttle; double rotation;} SC_DriveInput;
	
	enum DriveMode { DEFAULT, TANK, DIFFERENTIAL, MECANUM, SWERVE };
	enum SC_Wheel { FRONT_LEFT, FRONT_RIGHT, REAR_LEFT, REAR_RIGHT, LEFT_WHEEL, RIGHT_WHEEL };


	/*=======================*/
	/*=== Other Datatypes ===*/
	/*=======================*/
	template<class T>
	class SC_Array
	{
	public:
		SC_Array(uint size)
		{
			values = std::malloc(sizeof(T) * size);
			len = size;
		}

		SC_Array(uint size, T &initial)
		{
			values = std::malloc(sizeof(T) * size);
			len = size;
		}

		~SC_Array()
		{
			delete values;
			len = 0;
		}

		template<typename T2>
		SC_Array<T> operator+=(T2 other)
		{
			for(uint i; i < len; i++) { values[i] += static_cast<T>(other); }
			return this;
		}

		template<typename T2>
		SC_Array<T> operator-=(T2 other)
		{
			for(uint i; i < len; i++) { values[i] -= static_cast<T>(other); }
			return this;
		}

		template<typename T2>
		SC_Array<T> operator*=(T2 other)
		{
			for(uint i; i < len; i++) { values[i] *= static_cast<T>(other); }
			return this;
		}

		template<typename T2>
		SC_Array<T> operator/=(T2 val)
		{
			if(val = 0)
			{
				throw std::exception("Divide by zero error!");
				return this;
			}
			else
			{
				for(uint i = 0; i < this->len; i++) { this->values[i] /= static_cast<T>(val); }
				return this;
			}
		}

		SC_Array<T> operator[](int i)
		{
			if(std::abs(i) > this->len)
			{
				throw std::out_of_range("Index out of range! (SC_Array)");
				return this;
			}

			// Index from the back of the array, else normal indexing.
			if(i < 0) { return this->values[this-len - std::abs(i)]; }
			else { return this->values[i]; }
		}


	private:
		T *values;
		uint len;

	};
}

#endif //SC_DATATYPES_H
