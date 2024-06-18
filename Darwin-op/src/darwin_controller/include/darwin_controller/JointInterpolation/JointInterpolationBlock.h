#pragma once
#include <iostream>
#include <LBlocks/LBlocks.hpp>
#include <Eigen/Dense>
#include <tools/JointOrder.hpp>
#include <LSimpleRoboticsMath/Kinematics.hh>
#include <tools/StateFlag.h>

#define _JOINT_INP_BEGIN namespace ljh{namespace ctrl{namespace joint_interpolation{
#define _JOINT_INP_END }}}

_JOINT_INP_BEGIN

constexpr int JOINT_NUM = lee::tools::JointNumber;
struct Input{
    const int *pPressKey;
    const double *JointRealPos;
};
struct Output{
    double *JointRefPos;
};

// This block is used to do a second interpolation of all joint after the JointReset
class Block:public lee::blocks::LBlock<Input,Output>
{
protected:
    double Time, Ts;
    double MoveTimeS;
    lee::math::kinematics::State<JOINT_NUM> Joint;
    lee::math::kinematics::Vec<JOINT_NUM> JointTargetPos;
    bool IsRunning;

    ljh::tools::GUIStateFlag GUIFlag;

public:
    inline Block(const double &_Ts=0.001){
        this->BlockName = "JointInterpolation";
        this->Time = 0.0;
        this->Ts = _Ts;
        this->IsRunning = false;
        this->GUIFlag = ljh::tools::GUIStateFlag::GUI_OFF;
        std::cout<<"Block JointInterpolation Created"<<std::endl;
    };
    int init();
    int run();
    int print();
    int log();
    inline void setGUIFlag(const ljh::tools::GUIStateFlag &_flag){this->GUIFlag = _flag;};

    inline void setMoveTimes(const double& _Times){this->MoveTimeS = _Times;};
    void setJointTargetPosAll(double *_TargetPos);
    void setSingleJointTargetPos(int _JointOrder, const double& _TargetPos);
};



_JOINT_INP_END