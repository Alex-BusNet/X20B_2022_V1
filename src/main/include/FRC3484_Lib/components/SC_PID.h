#ifndef SC_PID_H
#define SC_PID_H

/*
 *   Desc: Custom PID Controller
 * Author: Port
 *   Date: 7/19/2020
 */


#include "FRC3484_Lib/utils/SC_Datatypes.h"
#include "FRC3484_Lib/utils/SC_Functions.h"
#include "units/time.h"

namespace SC
{
    /**
     * @brief      Custom PID controller. This controller assumes the
     *             Process Variable (PV) and Setpoint (SP) values are scaled 
     *             to be between 0% output and 100% output and are internally
     *             limited to remain between these bounds. Additionally, the 
     *             PID constants are also limited to be between 0 and 100.
     */
    class SC_PID
    {
    public:
        SC_PID();
        SC_PID(SC::SC_PIDConstants PIDc);
        SC_PID(SC::SC_PIDConstants PIDc, SC::SC_PID_AW_MODE awMode);
        ~SC_PID();

        void SetPIDConstants(SC::SC_PIDConstants PIDc);
        void SetPIDConstants(double nKp, double nKi, double nKd, double nKf);

        void SetKp(double nKp);
        void SetKi(double nKi);
        void SetKd(double nKd);
        void SetKf(double nKf);
        void SetSP(double nSP);

        /**
         * @brief   Sets the limits around where the SP is allowed to be set.
         */
        void SetSPLimits(double minSP, double maxSP);
        void SetSPLimits(SC::SC_Range<double> SPR);

        /**
         * @brief   Sets the limits around where the PV should be operating.
         */
        void SetPVLimits(double minPV, double maxPV);
        void SetPVLimits(SC::SC_Range<double> PVR);

        /**
         * @brief   Sets the limit of how much of the integral term is used.
         *          Value should be in the range of 0 - 1.
         */
        void SetILimits(double minI, double maxI); 
        void SetILimits(SC::SC_Range<double> IR);

        /**
         * @brief   Sets the limit of how much of the derivative term is used.
         *          Value should be in the range of 0 -1 .
         */
        void SetDLimits(double minD, double maxD);
        void SetDLimits(SC::SC_Range<double> DR);

        /**
         * @brief   Sets the Integral Anti-Windup method.
         */
        //void SetIAWMode(SC_PID_AW_MODE awMode) { this->antiwindupMode = awMode; }

        /**
         * @brief   Sets the tolerance window for when the integration is active
         *          (conditional integration anti-windup method only).
         */
        //void SetCITol(double pctTol) { this->tol = pctTol; }

        /**
         * @brief   Sets the Integral action time. Derivative filter 
         *          times are calculated from this when Enable() is called;
         */
        void Setdt(units::time::second_t ndt);

        /**
         * @brief   Enables the PID Controller. By default the 
         *          controller runs in standard mode.
         */
        void Enable();

        /**
         * @brief       Enables the PID Controller in
         *              inverted mode (100% CV = closed/stopped)
         * 
         * @param[in]   reverse     Reverse the error calculation.
         */
        void Enable(bool reverse);

        /**
         * @brief   Disables the PID controller. Disable() calls the
         *          Reset() function internally.
         *          
         */
        void Disable();

        /**
         * @brief   Enables Manual mode of the PID controller. Manual Mode holds the 
         *          CV at the last calculated value, and allows the CV to be overridden.
         */
        void EnableManualMode();

        /**
         * @brief   Turns off manual mode. The CV retains its value.
         */
        void DisableManualMode();

        /**
         * @brief       Allows the CV to be set manually. If the controller is
         *              not in manual mode, this function does nothing. 
         * 
         * @param nCV   New value for the CV.
         */
        void SetCV(double nCV);

        /**
         * @brief   Clears P, I, D, Error, CV, PV, and SP terms. 
         */
        void Reset();

        /**
         * @brief       Calculates the next CV term using the
         *              configured SP.
         * 
         * @param[in]   nPV     Process Variable (PV) that should be used in the calculation
         */
        double Calculate(double nPV);

        /**
         * @brief       Calculates the next CV term using the
         *              configured SP.
         * 
         * @param[in]   nPV     Process Variable (PV) that should be used in the calculation
         * @param[in]   FB      Control Variable (CV) feedback.
         */
        double Calculate(double nPV, double FB);

        /**
         * @brief       Overloaded declaration of Calculate(double nPV)
         * 
         * @param[in]   nPV     Process Variable (PV) that should be used in the calculation
         * @param[in]   FB      CV feedback. Used for integrator anti-windup.
         * @param[in]   nSP     The new SP to target
         */
        double Calculate(double nPV, double FB, double nSP);

        /**
         * @brief   Returns the current error of the controller.
         */
        double GetError();

        /**
         * @brief   Returns the current values of the P, I, D, Err,
         *          lastErr, enabled, and reversed statuses. This 
         *          does not return the current CV value, and should be 
         *          used for debugging purposes.
         */
        SC::SC_PIDStatus GetStatus();

        /**
         * @brief   Returns the enabled (running) status of the PID controller.
         */
        bool IsEnabled();

        /**
         * @brief   Returns the state of the PID controller (auto/manual).
         */
        bool InManualMode();

    private:
    // Functions
        /**
         * @brief   Calculates the new CV based on the current PV and SP.
         * 
         * @returns The new CV value
         */
        double Calculate();

        /**
         * @brief   Calculates the Ingerator Anti-Windup Coeff used with
         *          the back-calculation method.
         */
        void SetAntiWindupCoeff();

        /**
         * @brief   Calculates the derivative filter terms.
         */
        void SetDFilterTerms();

    // Members
        double Kp; // Proportional Coeff;
        double Ki; // Integral Coeff
        double Kd; // Derivative Coeff
        double Kf; // Feedforward Coeff
        double Kw; // Integral Anti-Windup Coeff (calculated value)
        double dt; // Integral action time

        double DervA, DervB; // Derivative filter terms.
        double err, lastErr;
        double I, P, D;

        SC::SC_Range<double> SCR_Integral, SCR_Deriv;
        SC::SC_Range<double> SCR_Setpoint, SCR_Process;
        double SP, PV, CV, CVfb;
        double manRate, trackerCV, manualCV;

        bool enabled, manualMode;
        bool reversed;
        SC::SC_PID_AW_MODE antiwindupMode;
    };
}

#endif