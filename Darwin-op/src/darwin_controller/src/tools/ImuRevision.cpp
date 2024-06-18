#include <tools/ImuRevision.h>
#include <Eigen/Dense>
#include <cmath>

#include <iostream>

namespace lee{namespace imu_revise{
    Eigen::Matrix3d wo_R_imuo = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d imu_R_w = Eigen::Matrix3d::Identity();

    auto rotX(const double _Q)
    {
        return Eigen::AngleAxisd(_Q, Eigen::Vector3d::UnitX()).matrix();
    }
    
    auto rotY(const double _Q)
    {
        return Eigen::AngleAxisd(_Q, Eigen::Vector3d::UnitY()).matrix();
    }
    
    auto rotZ(const double _Q)
    {
        return Eigen::AngleAxisd(_Q, Eigen::Vector3d::UnitZ()).matrix();
    }

    void transRot2Eul(const Eigen::Matrix3d &_Rot, double * _Q)
    {
        // Rz*Ry*Rx
        _Q[2] = atan2(_Rot(1,0), _Rot(0,0));
        _Q[0] = atan2(_Rot(2,1), _Rot(2,2));
        _Q[1] = -asin(_Rot(2,0));
    };
}}

void getPostureFromCopp(const double * _CoppPosture, double * _Posture)
{
    using namespace lee::imu_revise;
    Eigen::Matrix3d R = rotX(_CoppPosture[0])*rotY(_CoppPosture[1])*rotZ(_CoppPosture[2]);
    transRot2Eul(R, _Posture);
    // for(int i=0;i<3;i++) _Posture[i] = _CoppPosture[i];
}

void getPostureFromBHR(const double * _BHR7P3Posture, double * _Posture)
{
    // using namespace lee::imu_revise;
    // imu_R_w(0,0) = -1.0;
    // imu_R_w(2,2) = -1.0;

    // Eigen::MatrixXd imuo_R_imu = rotZ(_BHR7P3Posture[2])*rotY(_BHR7P3Posture[1])*rotX(_BHR7P3Posture[0]);
    // Eigen::Matrix3d wo_R_w = wo_R_imuo*imuo_R_imu*imu_R_w;
    // transRot2Eul(wo_R_w, _Posture);

    // Translation the posture in IMU data read code: rtx_main.cpp
    for(int i=0;i<3;i++)
    {
        _Posture[i] = _BHR7P3Posture[i];
    }
}

