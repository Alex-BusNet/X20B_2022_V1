#include "FRC3484_Lib/components/SC_PID.h"
#include "FRC3484_Lib/utils/SC_Functions.h"

using namespace units::time;
using namespace SC;

//--------------------
//--- Constructors ---
//--------------------
SC_PID::SC_PID()
{
    this->SetPIDConstants(1.0, 0.0, 0.0, 0.0);
    this->dt = 0.5; // 500 ms

    this->Reset(); 
    this->lastErr = 0.0;
    
    this->SCR_Integral(0.0, 100.0);
    this->SCR_Deriv(0.0, 100.0);
    this->SCR_Setpoint(-100.0, 100.0);
    this->SCR_Process(-100.0, 100.0);

    this->DervA = 0.0;
    this->DervB = 0.0;

    this->enabled = false;
    this->reversed = false;
    this->antiwindupMode = SC_PID_AW_MODE::OFF;
}

SC_PID::SC_PID(SC_PIDConstants PIDc)
{

    this->SetPIDConstants(PIDc);       
    this->dt = 0.5; // 500 ms
    
    this->Reset();
    this->lastErr = 0.0;
    
    this->SCR_Integral(0.0, 100.0);
    this->SCR_Deriv(0.0, 100.0);
    this->SCR_Setpoint(-100.0, 100.0);
    this->SCR_Process(-100.0, 100.0);

    this->DervA = 0.0;
    this->DervB = 0.0;

    this->enabled = false;
    this->reversed = false;

    this->antiwindupMode = SC_PID_AW_MODE::OFF;
}

SC_PID::SC_PID(SC_PIDConstants PIDc, SC_PID_AW_MODE awMode)
{
    this->SetPIDConstants(PIDc);
    this->dt = 0.5; // 500 ms
    
    this->Reset();

    this->lastErr = 0.0;
    
    this->SCR_Integral(0.0, 100.0);
    this->SCR_Deriv(0.0, 100.0);
    this->SCR_Setpoint(-100.0, 100.0);
    this->SCR_Process(-100.0, 100.0);

    this->DervA = 0.0;
    this->DervB = 0.0;

    this->enabled = false;
    this->reversed = false;
    this->antiwindupMode = awMode;

}

SC_PID::~SC_PID()
{
    // Dec'tor
    //
    // While this doesn't do any cleanup
    // this should be here to make sure 
    // any calls the runtime may make of it
    // are properly handled.
}

//------------------------
//--- Public Functions ---
//------------------------

void SC_PID::SetPIDConstants(SC_PIDConstants PIDc)
{
    this->Kp = F_Limit(0.0, 100.0, std::abs(PIDc.Kp));
    this->Ki = F_Limit(0.0, 100.0, std::abs(PIDc.Ki));
    this->Kd = F_Limit(0.0, 100.0, std::abs(PIDc.Kd));
    this->Kf = F_Limit(0.0, 100.0, std::abs(PIDc.Kf));
    this->SetAntiWindupCoeff(); 
}

void SC_PID::SetPIDConstants(double nKp, double nKi, double nKd, double nKf)
{
    this->Kp = F_Limit(0.0, 100.0, std::abs(nKp));
    this->Ki = F_Limit(0.0, 100.0, std::abs(nKi));
    this->Kd = F_Limit(0.0, 100.0, std::abs(nKd));
    this->Kf = F_Limit(0.0, 100.0, std::abs(nKf));
    this->SetAntiWindupCoeff(); 
}

void SC_PID::SetKp(double nKp)
{
    this->Kp = F_Limit(0.0, 100.0, nKp);
    this->SetAntiWindupCoeff();
}

void SC_PID::SetKi(double nKi)
{
    this->Ki = F_Limit(0.0, 100.0, nKi);
    this->SetAntiWindupCoeff();
}

void SC_PID::SetKd(double nKd)
{
    this->Kd = F_Limit(0.0, 100.0, nKd);
    this->SetAntiWindupCoeff();
}

void SC_PID::SetKf(double nKf)
{
    this->Kf = F_Limit(0.0, 100.0, nKf);
    this->SetAntiWindupCoeff();
}

void SC_PID::SetSP(double nSP) { this->SP = F_Limit(this->SCR_Setpoint, nSP); }

void SC_PID::SetSPLimits(double minSP, double maxSP) { this->SCR_Setpoint(minSP, maxSP); }

void SC_PID::SetSPLimits(SC_Range<double> SPR) { SetSPLimits(SPR.Val_min, SPR.Val_max); }

void SC_PID::SetPVLimits(double minPV, double maxPV) { this->SCR_Process(minPV, maxPV); }

void SC_PID::SetPVLimits(SC_Range<double> PVR) { SetPVLimits(PVR.Val_min, PVR.Val_max); }

void SC_PID::SetILimits(double minI, double maxI) { this->SCR_Integral(minI, maxI); }

void SC_PID::SetILimits(SC_Range<double> IR) { SetILimits(IR.Val_min, IR.Val_max); }

void SC_PID::SetDLimits(double minD, double maxD) { this->SCR_Deriv(minD, maxD); }

void SC_PID::SetDLimits(SC_Range<double> DR) { SetDLimits(DR.Val_min, DR.Val_max); }

void SC_PID::Setdt(units::time::second_t ndt) { this->dt = ndt.to<double>(); }

void SC_PID::Enable() { this->Enable(false); }

void SC_PID::Enable(bool reverse)
{
    this->enabled = true;
    this->reversed = reverse;
    
    this->SetDFilterTerms();
}

void SC_PID::Disable()
{
    this->enabled = false;
    this->Reset();
}

void SC_PID::EnableManualMode() { this->manualMode = true; }

void SC_PID::DisableManualMode() { this->manualMode = false; }

void SC_PID::SetCV(double nCV) { this->manualCV = F_Limit(0.0, 100.0, nCV); }

void SC_PID::Reset()
{
    this->CV = 0.0;
    this->SP = 0.0;
    this->PV = 0.0;

    this->P = 0.0;
    this->I = 0.0;
    this->D = 0.0;
    this->err = 0.0;
}

double SC_PID::Calculate(double nPV)
{
    this->PV = F_Limit(SCR_Process, nPV);
    this->CVfb = CV;
    return this->Calculate();
}

double SC_PID::Calculate(double nPV, double FB)
{
    this->PV = F_Limit(SCR_Process, nPV);
    this->CVfb = FB;
    return this->Calculate();
}

double SC_PID::Calculate(double nPV, double FB, double nSP)
{
    this->SP = F_Limit(SCR_Setpoint, nSP);
    return this->Calculate(nPV, FB);
}

double SC_PID::GetError() { return this->err; }

SC_PIDStatus SC_PID::GetStatus()
{
    return SC_PIDStatus{P, I, D, err, lastErr, 
                        antiwindupMode, Kw, enabled, 
                        reversed, SP};
}

bool SC_PID::IsEnabled() { return this->enabled; }

bool SC_PID::InManualMode() { return this->manualMode; }

//-------------------------
//--- Private Functions ---
//-------------------------
double SC_PID::Calculate()
{
    if(enabled)
    {
        if(!manualMode)
        {
            if(reversed)
                err = PV - SP;
            else
                err = SP - PV;

            // Calculate the Proportional term
            P = Kp * err;

            // Calculate the Integral term. 
            I = I + (Ki * err * dt) + ((CVfb - CV) * Kw);
            I = F_Limit(SCR_Integral, I);

            // Calculate the Derivative term
            D = (DervA * D) + (DervB * ((Kd * (err - lastErr)) / dt));
            D = F_Limit(SCR_Deriv, D);

            lastErr = err;

            // Calculate the output
            CV = F_Limit(0.0, 100.0, (SP * Kf) + P + I + D);
            
            // Used to hold the CV in manual mode
            trackerCV = CV;
            manualCV = trackerCV;

            return CV;
        }
        else
        {
            // ramp the CV up to the new CV
            if(manualCV > trackerCV)
            {
                trackerCV = F_Limit(trackerCV, manualCV, trackerCV + manRate * dt);
            }

            // Ramp the CV down to the new CV
            if(manualCV < trackerCV)
            {
                trackerCV = F_Limit(trackerCV, manualCV, trackerCV - manRate * dt);
            }

            trackerCV = F_Limit(SCR_Integral, trackerCV);

            // Set the integral value to the CV/
            I = trackerCV;
            
            // Reset derivative gain
            D = 0;

            // Set the output
            CV = trackerCV;

            return CV;
        }
    }
    else
    {
        P = 0;
        I = 0;
        D = 0;
        CV = 0;
        trackerCV = 0;
        err = 0;
        lastErr = 0;

        return 0;
    }
}

void SC_PID::SetAntiWindupCoeff()
{
    // Ki = Kp / Ti => Ti = Kp / Ki
    // Kd = Kp * Td => Td = Kd / Kp

    // No derivative term in use
    if((this->Kd == 0.0) && (this->Kp > 0.0))
    {
        // Kw = 1 / Ti 
        // => Kw = 1 / (Kp / Ki) 
        // => Kw = Ki / Kp
        this->Kw = this->Ki / this->Kp;
    }
    else if((this->Kd > 0.0) && (this->Ki > 0.0))
    {
        // Kw = 1 / SQRT(Ti * Td) 
        // => Kw = 1 / SQRT (Kp/Ki * Kd/Kp)
        // => Kw = 1 / SQRT(Kd / Ki)
        this->Kw = 1 / std::sqrt(this->Kd / this->Ki);
    }
    else
    {
        this->Kw = 0.0;
    }
}

void SC_PID::SetDFilterTerms()
{
    this->DervB = dt / 1000.0;
    this->DervA = 1.0 - DervB;
}