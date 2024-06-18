#include "Base.h"
#include "Math.hpp"
_THESIS_TOOL_BEGIN

namespace zmp{

using math::Vec3;
using math::MatT;
using math::RotM;

inline auto calFootZMP(const double *f, const double *tau, const double &_Threshold=20.0)
{
    Vec3 res = {0,0,0};
    if(f[2] < _Threshold) return res;

    double px = 0.0, py = 0.0, pz = 0.03;
    res(0) = (-tau[1] - pz * f[0] + px * f[2]) / f[2];
    res(1) = ( tau[0] - pz * f[1] + py * f[2]) / f[2];
    return res;
}

inline auto calLocalZMP(
    const Vec3 &FootZMP_L,   const Vec3 &FootZMP_R, 
    const Vec3 &W_FootL_Pos, const Vec3 &W_FootL_Ang,
    const Vec3 &W_FootR_Pos, const Vec3 &W_FootR_Ang,
    const Vec3 &W_Local_Pos, const Vec3 &W_Local_Ang,
    const double &FzL, const double &FzR, const double &_Threshold=20.0)
{
    using namespace math;
    Vec3 res = {0,0,0};
    
    if(FzL+FzR < _Threshold) return res;

    // Calculate Foot ZMP in Local Frame
    MatT Foot_T_ZMP[2] = {Move(FootZMP_L.data()), Move(FootZMP_R.data())};
    MatT W_T_Foot[2] = {
        Move(W_FootL_Pos.data()) * RotZYX(W_FootL_Ang.data()),
        Move(W_FootR_Pos.data()) * RotZYX(W_FootR_Ang.data())
    };
    MatT L_T_W = (Move(W_Local_Pos.data()) * RotZ(W_Local_Ang(2))).inverse();
    MatT L_T_SingleZMP[2];
    for (int i = 0; i < 2; i++)
    {
        L_T_SingleZMP[i] = L_T_W * W_T_Foot[i] * Foot_T_ZMP[i];
    }

    // Calculate Local ZMP in Local Frame
    Vec3 ZMP_L = L_T_SingleZMP[0].block<3, 1>(0, 3);
    Vec3 ZMP_R = L_T_SingleZMP[1].block<3, 1>(0, 3);
    res    = FzL / (FzL + FzR) * ZMP_L + FzR / (FzL + FzR) * ZMP_R;
    res(2) = 0.5*(ZMP_L(2) + ZMP_R(2));

    return res;
}

}

_THESIS_TOOL_END