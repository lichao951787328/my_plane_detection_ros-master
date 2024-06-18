// DCM_Walk/PlanBody.hpp 
// lee, hexb66@bit.edu.cn
// Mar. 13, 2022
#pragma once
#include "Base.h"
_DCM_BEGIN

class BodyPlanner
{
protected:
    State3 BodyAng;
    State3 BodyPos;
    Vec3   CoM_Bias;

public:
    inline BodyPlanner(const Vec3 &_CoM_Bias = {0.0, 0.0, 0.0}){
        this->CoM_Bias = _CoM_Bias;
    };
    inline auto &getBodyAng(){return this->BodyAng;};
    inline auto &getBodyPos(){return this->BodyPos;};
    inline auto &getCoM_Bias(){return this->CoM_Bias;};
    
    // Call calBldyAng First!!!
    inline void calBodyPos(const State3 &_CoM)
    {
        this->BodyPos = _CoM;
        // Notify: Roll and Pitch are both zero here
        Eigen::Matrix3d Rot = Eigen::AngleAxisd(this->BodyAng.Pos(2), Vec3::UnitZ()).matrix();
        this->BodyPos.Pos = _CoM.Pos - Rot*this->CoM_Bias;
    };
    inline void calBodyAng(const State3 &_FootAngL, const State3 &_FootAngR)
    {
        this->BodyAng.Pos(2) = 0.5*(_FootAngL.Pos(2)+_FootAngR.Pos(2));
        this->BodyAng.Vel(2) = 0.5*(_FootAngL.Vel(2)+_FootAngR.Vel(2));
        this->BodyAng.Acc(2) = 0.5*(_FootAngL.Acc(2)+_FootAngR.Acc(2));
    };
};

_DCM_END