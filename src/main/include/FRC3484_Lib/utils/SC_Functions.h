#ifndef SC_FUNCTIONS_H
#define SC_FUNCTIONS_H

#include "FRC3484_Lib/utils/SC_Datatypes.h"

namespace SC
{
    
    //==============================
    // F_Limit Declarations
    // --------------------
    //
    // F_Limit is declared with 1 
    // overload declaration to allow 
    // usage of SC_Range datatype.
    //===============================

    /**
     * @brief   Limits the value of the input, fin, to 
     *          the defined bounds, [fmin, fmax]. The
     *          input is returned unmodified if it is
     *          along (fmin, fmax).
     */

    /*
    template<class T>
    T F_Limit(T fmin, T fmax, T fin);


    template<class T>
    T F_Limit(SC_Range<T> valRange, T fin);
    */

    template<class T>
    T F_Limit(T fmin, T fmax, T fin)
    {
        // This looks a little odd, but since fmin is
        // larger than fmax, we want to reverse which
        // one is treated as the larger or smaller value
        if(fmin > fmax)
            return std::max(fmax, std::min(fmin, fin));
        else
            return std::max(fmin, std::min(fmax, fin));

    }

    template<class T>
    T F_Limit(SC::SC_Range<T> valRange, T fin)
    {
        return std::max(valRange.Val_min, std::min(valRange.Val_max, fin));
    }

    /**
     * @brief   Overloaded declaration of F_Limit.
     *          "bounded" is set to true if fin is
     *          outside the defined limits.
     */
    template<class T>
    T F_Limit(T fmin, T fmax, T fin, bool& bounded)
    {
        T _min, _max;

        if(fmin > fmax)
        { 
            _min = fmax;
            _max = fmin;
        }
        else
        {
            _min = fmin;
            _max = fmax;
        }

        if(fin > _max)
        {
            bounded = true;
            return _max;
        }
        else if(fin < _min)
        {  
            bounded = true;
            return _min;
        }
        else
        {
            bounded = false;
            return fin;
        }
    }

    template<class T>
    T F_Limit(SC_Range<T> valRange, T fin, bool& bounded)
    {
        if(fin > valRange.Val_max)
        {
            bounded = true;
            return valRange.Val_max;
        }
        else if(fin < valRange.Val_min)
        {
            bounded = true;
            return valRange.Val_min;
        }
        else
        {
            bounded = false;
            return fin;
        }
    }
    
    //=========================================
    // F_Scale declarations
    // --------------------
    //
    // F_Scale is declared with 4 
    // overloaded method calls allowing
    // a combination of datatypes and SC_Range
    // instances to be used for data scaling.
    //=========================================

    /**
     * @brief   F_Scale provides converts the input defined
     *          along [InMin, InMax] to an output defined
     *          along [OutMin, OutMax].
     */
    template<class T1, class T2>
    T2 F_Scale(T1 InMin, T1 InMax, T2 OutMin, T2 OutMax, T1 fin)
    {
        if((InMax - InMin) != 0)
        {
            T2 result = (((fin -InMin)/ (InMax - InMin)) * (OutMax - OutMin)) + OutMin;
            return SC::F_Limit(OutMin, OutMax, result);
        }
        else
            return 0;    
    }

    template<class T1, class T2>
    T2 F_Scale(SC_Range<T1> InRange, T2 OutMin, T2 OutMax, T1 fin)
    {
        return SC::F_Scale(InRange.Val_min, InRange.Val_max, OutMin, OutMax, fin);
    }


    template<class T1, class T2>
    T2 F_Scale(T1 InMin, T1 InMax, SC_Range<T2> OutRange, T1 fin)
    {
        return SC::F_Scale(InMin, InMax, OutRange.Val_min, OutRange.Val_max, fin);
    }

    template<class T1, class T2>
    T2 F_Scale(SC_Range<T1> InRange, SC_Range<T2> OutRange, T1 fin)
    {
        return SC::F_Scale(InRange.Val_min, InRange.Val_max, OutRange.Val_min, OutRange.Val_max, fin);
    }

    //=================
    // Misc. Functions
    //=================
    template<class T>
    double F_Percent(T num, T den)
    {
        if(den != static_cast<T>(0))
            return ((double)num / (double)den) * 100.0;
        else
            return 0;
    }


    /**
     * @brief   F_XYCurve provides a single interpolated output
     *          value based on a set of X and Y datasets that 
     *          characterize the dynamics of a system. 
     * 
     *          If the input value is lower than the first value
     *          of `known_xs`, then the function returns 
     *          `known_ys[0]`. If the input value is larger than
     *          `known_xs[size]`, then the function returns `known_ys[size]`.
     * 
     *          `known_xs` and `known_ys` must be the size size
     *          sorted in ascending order.
     * 
     * @param   known_xs Array of input (X) values that characterize the system.
     * 
     * @param   known_ys Array of output (Y) values that characterize the response
     *                   to each input (X) value. Must be the same size as `known_xs`.
     * 
     * @param   xin      The input value to be calculated against.
     * 
     * @param   size     The number of elements in `known_xs` and `known_ys`
     */
    template<class T>
    T F_XYCurve(T* known_xs, T* known_ys, T xin, int size)
    {
        if((size < 2) || (xin <= known_xs[0]))
            return known_ys[0];
        else if(xin >= known_xs[size-1])
            return known_ys[size-1];
        else
        {
            int idx = 0;

            // Search for which x values surround the input value
            // Check is index + 1, so the loop should stop at the
            // second-to-last element in the array
            for(idx = 0; idx < size-1; idx++)
            {
                if((known_xs[idx] <= xin) && (known_xs[idx+1] > xin))
                    break;
            }

            return known_ys[idx] + (xin - known_xs[idx]) * ((known_ys[idx+1] - known_ys[idx]) / (known_xs[idx+1] - known_xs[idx]));
        }   
    }

    template<class T>
    T F_Deadband(T fin, T deadband)
    {
        return (fin >= deadband) ? fin : 0;
    }
}
#endif // SC_FUNCTIONS_H