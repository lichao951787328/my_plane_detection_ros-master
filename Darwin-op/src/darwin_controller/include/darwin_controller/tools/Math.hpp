#pragma once
#include "Base.h"
#include <LSimpleRoboticsMath/Kinematics.hh>
#include <Eigen/EigenKinematics.hh>

_THESIS_TOOL_BEGIN

namespace math{

using namespace eigen_kinematics;
using lee::math::kinematics::Vec3;
using lee::math::kinematics::Vec;
using lee::math::kinematics::State3;
using lee::math::kinematics::State;
using Eigen::MatT;
using RotM = Eigen::Matrix<double,3,3>;

inline RotM getRot(MatT & Mat){return Mat.block<3,3>(0,0);};
inline Vec3 getPos(MatT & Mat){return Mat.block<3,1>(0,3);};

// inline RotM RotX(const double &Ang)
// {
//     return Eigen::AngleAxisd(Ang, Vec3::UnitX()).matrix();
// };
// inline RotM RotY(const double &Ang)
// {
//     return Eigen::AngleAxisd(Ang, Vec3::UnitY()).matrix();
// };
// inline RotM RotZ(const double &Ang)
// {
//     return Eigen::AngleAxisd(Ang, Vec3::UnitZ()).matrix();
// };
// inline RotM RotZYX(const double *pAng)
// {
//     return RotZ(pAng[2])*RotY(pAng[1])*RotX(pAng[0]);
// };

inline RotM crossLeft(const double *vec)
{
    RotM res;
    auto &x = vec[0];
    auto &y = vec[1];
    auto &z = vec[2];
    res << 0, -z,  y,
           z,  0, -x,
          -y,  x,  0;
    return res;
};

inline RotM crossLeft(const Vec3 &vec)
{
    RotM res;
    auto &x = vec(0);
    auto &y = vec(1);
    auto &z = vec(2);
    res << 0, -z,  y,
           z,  0, -x,
          -y,  x,  0;
    return res;
};

inline bool isEqual(const double &Val1, const double &Val2, const double &Error=1e-6)
{
    if(abs(Val1-Val2)<Error)
        return true;
    else 
        return false;
}

}

_THESIS_TOOL_END