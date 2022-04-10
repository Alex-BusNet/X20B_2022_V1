
#include "units/time.h"

namespace SC
{
	/**
	 * Alpha-Beta Filter templated against the units library.
	 */
	template<class T>
	class SC_ABFilterU
	{
	public:
		SC_ABFilterU<T>(units::time::second_t FilterTime, units::time::second_t ScanTime)
		{
			this->_tau = FilterTime.value();
			this->_scanT = ScanTime.value();
		}

		~SC_ABFilterU<T>()
		{
			;
		}

		T Filter(T PV)
		{
			T pvf = PV; // Filtered input value

			if(this->_tau != this->_scanT)
			{
				this->_beta = this->_scanT / (this->_tau + this->_scanT);
				this->_alpha = 1 - this->_beta;

				pvf = (this->_alpha * PV_out) + (this->_beta * PV);

				PV_out = units::math::abs(pvf).value() < 1E-9 ? units::make_unit<T>(0.0) : pvf;

				return PV_out;

			}

			return pvf;
		}
	private:
		double _tau, _scanT;
		double _alpha, _beta;
		T PV_out;

	};

	/**
	 * Alpha-Beta Filter templated against C++ standard library
	 */
	template<class T>
	class SC_ABFilter
	{
	public:
		SC_ABFilter<T>(units::time::second_t FilterTime, units::time::second_t ScanTime)
		{
			this->_tau = FilterTime.value();
			this->_scanT = ScanTime.value();
		}

		~SC_ABFilter<T>()
		{
			;
		}

		T Filter(T PV)
		{
			T pvf = PV; // Filtered input value

			if(this->_tau != this->_scanT)
			{
				this->_beta = this->_scanT / (this->_tau + this->_scanT);
				this->_alpha = 1 - this->_beta;

				pvf = (this->_alpha * PV_out) + (this->_beta * PV);

				PV_out = std::abs(pvf) < 1E-9 ? 0.0 : pvf;

				return PV_out;

			}

			return pvf;
		}
	private:
		double _tau, _scanT;
		double _alpha, _beta;
		T PV_out;

	};
}