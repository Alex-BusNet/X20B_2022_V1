//#include "FRC3484_Lib/utils/SC_Functions.h"
//#include "units/math.h"

/*
 * F_Limit body declarations
 */
/*
template<class T>
T SC::F_Limit(T fmin, T fmax, T fin)
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
T SC::F_Limit(SC::SC_Range<T> valRange, T fin)
{
    return std::max(valRange.Val_min, std::min(valRange.Val_max, fin));
}

template<class T>
T SC::F_Limit(T fmin, T fmax, T fin, bool& bounded)
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
T SC::F_Limit(SC::SC_Range<T> valRange, T fin, bool& bounded)
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

/*
 * F_Scale body declarations
 */
/*
template<class T1, class T2>
T2 SC::F_Scale(T1 InMin, T1 InMax, T2 OutMin, T2 OutMax, T1 fin)
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
T2 SC::F_Scale(SC::SC_Range<T1> InRange, T2 OutMin, T2 OutMax, T1 fin)
{
    return SC::F_Scale(InRange.Val_min, InRange.Val_max, OutMin, OutMax, fin);
}

template<class T1, class T2>
T2 SC::F_Scale(T1 InMin, T1 InMax, SC::SC_Range<T2> OutRange, T1 fin)
{
    return SC::F_Scale(InMin, InMax, OutRange.Val_min, OutRange.Val_max, fin);
}

template<class T1, class T2>
T2 SC::F_Scale(SC::SC_Range<T1> InRange, SC::SC_Range<T2> OutRange, T1 fin)
{
    return SC::F_Scale(InRange.Val_min, InRange.Val_max, OutRange.Val_min, OutRange.Val_max, fin);
}
*/

/*
 * Misc. Function body declarations
 */
/*
template<class T>
double SC::F_Percent(T num, T den)
{
    if(den != static_cast<T>(0))
        return ((double)num / (double)den) * 100.0;
    else
        return 0;
}

template<class T>
T SC::F_XYCurve(T* known_xs, T* known_ys, T xin, int size)
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
*/