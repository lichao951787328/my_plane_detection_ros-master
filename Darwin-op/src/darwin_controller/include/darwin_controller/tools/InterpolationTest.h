#pragma once
#include <iostream>
#include <string>
#include <LBlocks/LBlocks.hpp>
#include <LSimpleRoboticsMath/Kinematics.hh>
#include <tools/StateFlag.h>
extern "C"{
    #include <LSimpleRoboticsMath/leeMatrix.h>
}
#define _INTERP_TEST_BEGIN namespace ljh{namespace tools{
#define _INTERP_TEST_END }}

_INTERP_TEST_BEGIN

enum CoMState{
    PosX,
    PosY,
    PosZ,
    Roll,
    Pitch,
    Yaw,
    CoMNUM
};

enum DunState{
    NONE,
    START,
    UP,
    DOWN,
    END
};

enum LimbState{
    LLeg,
    RLeg,
    LArm,
    RArm
};

constexpr int LIMB_NUM = 6;
struct Input{
    const int *pPressKey;
    const double *LArmJointRealPos;
    const double *RArmJointRealPos;
    const double *LLegJointRealPos;
    const double *RLegJointRealPos;
};
struct Output{
    double *LArmJointRefPos;
    double *RArmJointRefPos;
    double *LLegJointRefPos;
    double *RLegJointRefPos;
};

class Block:public lee::blocks::LBlock<Input,Output>
{
protected:
    double Time, Ts;
    double MoveTimeS;
    double SumMoveTimes;
    int DunCount;
    int DunCountMax;
    double Joint5UpDeg;
    double Joint5DownDeg;
    ljh::tools::GUIStateFlag GUIFlag;
    lee::math::kinematics::State<LIMB_NUM> LArmJoint;
    lee::math::kinematics::State<LIMB_NUM> RArmJoint;
    lee::math::kinematics::State<LIMB_NUM> LLegJoint;
    lee::math::kinematics::State<LIMB_NUM> RLegJoint;

    lee::math::kinematics::Vec<LIMB_NUM> LArmJointIni;
    lee::math::kinematics::Vec<LIMB_NUM> RArmJointIni;
    lee::math::kinematics::Vec<LIMB_NUM> LLegJointIni;
    lee::math::kinematics::Vec<LIMB_NUM> RLegJointIni;
    
    lee::math::kinematics::Vec<LIMB_NUM> LArmJointTargetPosUp;
    lee::math::kinematics::Vec<LIMB_NUM> RArmJointTargetPosUp;
    lee::math::kinematics::Vec<LIMB_NUM> LLegJointTargetPosUp;
    lee::math::kinematics::Vec<LIMB_NUM> RLegJointTargetPosUp;
    lee::math::kinematics::Vec<LIMB_NUM> LArmJointTargetPosDown;
    lee::math::kinematics::Vec<LIMB_NUM> RArmJointTargetPosDown;
    lee::math::kinematics::Vec<LIMB_NUM> LLegJointTargetPosDown;
    lee::math::kinematics::Vec<LIMB_NUM> RLegJointTargetPosDown;

    bool IsRunning;
    enum DunState DunStateFlag;

public:
    inline Block(const double &_Ts=0.001){
        this->BlockName = "JointInterpolation";
        this->Time = 0.0;
        this->Ts = _Ts;
        this->MoveTimeS = 5.0;
        this->SumMoveTimes = 5.0;
        this->DunCount = 0;
        this->DunCountMax = 0;
        this->IsRunning = false;
        this->DunStateFlag = NONE;
        this->Joint5DownDeg = -10.;
        this->Joint5UpDeg = -10.;
        std::cout<<"Block JointInterpolation Created"<<std::endl;
    };
    int init();
    int run();
    int print();
    int log();
    void InerpolationLimbJoint(lee::math::kinematics::Vec<LIMB_NUM> JointTargetPos, enum LimbState LimbFlag);
    std::string tostring(enum DunState DunFlag);
    inline void setGUIFlag(const ljh::tools::GUIStateFlag &_flag){this->GUIFlag = _flag;};
    inline void setDunCount(const int & _count){this->DunCountMax = _count;};
    inline void setMoveTime(const double & _time){this->MoveTimeS = _time; this->SumMoveTimes = _time;};
    inline void setJointPosUp5(const double & _up){this->Joint5UpDeg = _up;};
    inline void setJointPosDown5(const double & _down){this->Joint5DownDeg = _down;};
};

_INTERP_TEST_END