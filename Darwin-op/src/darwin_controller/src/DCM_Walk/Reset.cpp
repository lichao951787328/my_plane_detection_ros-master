#include <DCM_Walk/Block.h>
#include <DCM_Walk/Core.hpp>
#include <LSimpleRoboticsMath/Kinematics.hh>
#include <Eigen/EigenKinematics.hh>

namespace lee{namespace DCM_WalkPlanner{

extern dcm::DCM_Walk<> Planner;
extern dcm::Vec3 CoM_Bias, InitBodyPos, TarCoM_Pos, InitCoM_Pos;
extern double TarBodyHeight;

void Block::reset( 
    const double *_BodyPos,  const double *_BodyAng, 
    const double *_FootPosL, const double *_FootAngL, 
    const double *_FootPosR, const double *_FootAngR
    )
{
    using dcm::Vec3;
    using namespace eigen_kinematics;

    // Calculate init body and com pos
    InitBodyPos = dcm::Vec3{_BodyPos};
    Eigen::MatT w_t_body = Move(_BodyPos)*RotZYX(_BodyAng);
    Eigen::MatT b_t_com = Move(CoM_Bias.data());
    Eigen::MatT w_t_com = w_t_body*b_t_com;
    InitCoM_Pos = w_t_com.block<3,1>(0,3);
    
    // Calculate target com pos
    TarCoM_Pos(0) = 0.5*(_FootPosL[0]+_FootPosR[0]);
    TarCoM_Pos(1) = 0.5*(_FootPosL[1]+_FootPosR[1]);
    TarCoM_Pos(2) = 0.5*(_FootPosL[2]+_FootPosR[2])+TarBodyHeight+CoM_Bias(2);
    
    // Reset
    this->StateFlag = STATE_SQUAT;
    this->Time = 0.0;
    Planner.initPlanner(TarCoM_Pos, Vec3{_FootPosL}, Vec3{_FootPosR}, Vec3{_FootAngL}, Vec3{_FootAngR});
    Planner.getCoM().Pos = InitCoM_Pos;
}

}}