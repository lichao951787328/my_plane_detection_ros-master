// DCM_Walk/Base.hpp
// Basic type, constants, and functions used in DCM walk plan
// lee, hexb66@bit.edu.cn
// Mar. 06, 2022
#pragma once
#include <LSimpleRoboticsMath/leeMatrix++.hh>
#include <LSimpleRoboticsMath/Biped.hh>
#include <deque>

// Namespace is dcm, part of lee
#define _DCM_BEGIN namespace lee{namespace dcm{
#define _DCM_END }}

_DCM_BEGIN
using namespace lee::math::biped;

// Flag about robot state
enum _ROBOT_STATE{STAND, WALK_INIT, WALK_SS, WALK_DS};
// Flag about current step state
enum _CURRENT_STEP_STATE{STEP_CONTINUE, STEP_OVER};

// Calculate constant of first-order dynamics about DCM and CoM
inline double calDCM_ConstantB(const double &_Zc){return sqrt(_Zc/Gravity);};

// Calculate DCM state corresponding to constant VRP
// Position
template<typename _Type>
inline _Type calDCM_Pos(const _Type &_VRP, const _Type &_DCM, const double &_Time, const double &_B)
{   
    _Type res = _VRP + exp(_Time/_B)*(_DCM-_VRP);
    return res;
};
// Velocity
template<typename _Type>
inline _Type calDCM_Vel(const _Type &_VRP, const _Type &_DCM, const double &_Time, const double &_B)
{   
    _Type res = exp(_Time/_B)*(_DCM-_VRP)/_B;
    return res;
};
// Acceleration
template<typename _Type>
inline _Type calDCM_Acc(const _Type &_VRP, const _Type &_DCM, const double &_Time, const double &_B)
{   
    _Type res = exp(_Time/_B)*(_DCM-_VRP)/_B/_B;
    return res;
};

// Basic realtime interpolation function
template<const int N, const int INDEX=-1>
inline void calRealTimeInterpolation(State<N> &_Current, const State<N> &_Target, const double &_CurrentTime, const double &_GoalTime, const double &_Ts)
{
    if(INDEX==-1)
    {
        for (int i = 0; i < N; i++)
        {
            // realtime_1D_interpolation(&_Current.Pos(i), &_Current.Vel(i), _Target.Pos(i), _Target.Vel(i), _CurrentTime, _GoalTime, _Ts);
            realtime_1D_interpolation_5(&_Current.Pos(i), &_Current.Vel(i), &_Current.Acc(i), _Target.Pos(i), _Target.Vel(i), _Target.Acc(i), _CurrentTime, _GoalTime, _Ts);
        }
    }
    else if(INDEX<N && INDEX>=0)
    {
        int i=INDEX;
        // realtime_1D_interpolation(&_Current.Pos(i), &_Current.Vel(i), _Target.Pos(i), _Target.Vel(i), _CurrentTime, _GoalTime, _Ts);
        realtime_1D_interpolation_5(&_Current.Pos(i), &_Current.Vel(i), &_Current.Acc(i), _Target.Pos(i), _Target.Vel(i), _Target.Acc(i), _CurrentTime, _GoalTime, _Ts);
    }
    else
    {
        return;
    }
};

_DCM_END