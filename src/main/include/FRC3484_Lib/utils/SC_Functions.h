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
    template<class T>
    T F_Limit(T fmin, T fmax, T fin);


    template<class T>
    T F_Limit(SC_Range<T> valRange, T fin);

    /**
     * @brief   Overloaded declaration of F_Limit.
     *          "bounded" is set to true if fin is
     *          outside the defined limits.
     */
    template<class T>
    T F_Limit(T fmin, T fmax, T fin, bool& bounded);

    template<class T>
    T F_Limit(SC_Range<T> valRange, T fin, bool& bounded);
    
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
    T2 F_Scale(T1 InMin, T1 InMax, T2 OutMin, T2 OutMax, T1 fin);

    template<class T1, class T2>
    T2 F_Scale(SC_Range<T1> InRange, T2 OutMin, T2 OutMax, T1 fin);

    template<class T1, class T2>
    T2 F_Scale(T1 InMin, T1 InMax, SC_Range<T2> OutRange, T1 fin);

    template<class T1, class T2>
    T2 F_Scale(SC_Range<T1> InRange, SC_Range<T2> OutRange, T1 fin);


    //=================
    // Misc. Functions
    //=================
    template<class T>
    double F_Percent(T num, T den);

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
    T F_XYCurve(T* known_xs, T* known_ys, T xin, int size);
}
#endif // SC_FUNCTIONS_H