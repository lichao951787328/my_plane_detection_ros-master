#pragma once
#include "Base.h"
#include <cmath>

_THESIS_TOOL_BEGIN namespace control{

constexpr double Pi = 3.141592657;

/**
 * @brief Calculate PD control gains according to desired performance index of second order system 
 * 
 * @param _Mp Maximum overshoot 
 * @param _Me Steady-state error
 * @param _Ts Settling time 
 * @param Kp  Output: Proportional gain
 * @param Kd  Output: Differential gain
 */
inline void calPD_Gains(const double &_Mp, const double &_Me, const double &_Ts, double &Kp, double &Kd)
{
    double wn, xi;
    double lnmp = log(_Mp);
    xi = sqrt(lnmp*lnmp / (Pi*Pi + lnmp*lnmp));
    wn = -1 / (_Ts*xi)*log(_Me*sqrt(1 - xi*xi));
    Kp = wn*wn;
    Kd = 2.0*wn*xi;
}
} _THESIS_TOOL_END