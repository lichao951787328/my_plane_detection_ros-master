#pragma once
#include "Base.h"
#include "Math.hpp"

_THESIS_TOOL_BEGIN namespace type{

using lee::math::kinematics::Vec;

template<typename _T=double>
struct PtrJoint{
    _T *LegL, *LegR, *Waist, *ArmL, *ArmR;
};

template<typename _T=double>
struct PtrIMU{
    _T *Ang, *Omg, *Acc;
};

template<typename _T=double>
struct PtrForceSensor{
    _T *Force, *Torque;
};

struct Joint{
    Vec<6> LegL, LegR;
    Vec<1> Waist;
    Vec<3> ArmL;
    Vec<3> ArmR;

    template<typename _T=double>
    inline auto getPtr()
    {
        return PtrJoint<_T>({
            LegL.data(), 
            LegR.data(), 
            Waist.data(), 
            ArmL.data(), 
            ArmR.data()}
        );
    };
};

struct ForceSensor{
    Vec<3> Force, Torque;

    inline ForceSensor(){Force.setZero();Torque.setZero();};

    template<typename _T=double>
    inline auto getPrt(){return PtrForceSensor<_T>({Force.data(), Torque.data()});};
};

struct IMU{
    Vec<3> Ang, Omg, Acc;
    inline IMU(){Ang.setZero();Omg.setZero();Acc.setZero();};

    template<typename _T=double>
    inline auto getPtr(){return PtrIMU<_T>({Ang.data(), Omg.data(), Acc.data()});};
};

struct DynamicsReference
{
    ForceSensor FT_L, FT_R;
    ForceSensor FT_Vel_L, FT_Vel_R;
    Vec<3> LocalZMP;
};

}_THESIS_TOOL_END