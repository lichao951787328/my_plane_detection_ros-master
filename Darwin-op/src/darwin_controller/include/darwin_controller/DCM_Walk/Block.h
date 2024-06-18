// DCM_Walk/Block.h
// lee, hexb66@bit.edu.cn
// Mar.14, 2022
#pragma once
#include <LBlocks/LBlocks.hpp>
#include <LBlocks/LLog.hpp>
#include "Base.h"
#include <tools/StateFlag.h>
namespace lee{namespace DCM_WalkPlanner{
enum STATE_FLAG{STATE_IDLE, STATE_SQUAT, STATE_DCM_WALK};
class Input
{
public:
    const int *PressKey;
    std::deque<lee::dcm::FootholdType> *pFootholdList;
};

class Output
{
public:
    double 
        *RefBodyPos, 
        *RefBodyAng, 
        *RefFootPosL, 
        *RefFootPosR, 
        *RefFootAngL, 
        *RefFootAngR,
        *RefZMP,
        *RefCoM;
    double 
        *RefBodyLinearVel, 
        *RefBodyAngularVel, 
        *RefBodyLinearAcc;
    double 
        *RefFootLinearVel_L, 
        *RefFootLinearAcc_L, 
        *RefFootAngVel_L, 
        *RefFootAngAcc_L;
    double 
        *RefFootLinearVel_R, 
        *RefFootLinearAcc_R, 
        *RefFootAngVel_R, 
        *RefFootAngAcc_R;
    int *WalkState;

    int SupFlag;

    double *RefDCM, *RefDCM_Vel, *RefDCM_Acc;
    double *pTimeInStep;
    int *StepNo;
};

class Block: public lee::blocks::LBlock<Input,Output>
{
public:
    Block();
    int init();
    int run();
    int clear();
    int print();
    int log();

    void reset(
        const double *_BodyPos,  const double *_BodyAng, 
        const double *_FootPosL, const double *_FootAngL, 
        const double *_FootPosR, const double *_FootAngR
        );

    void setTarBodyHeight(const double &_Height);
    void setCoM_Bias(const dcm::Vec3 &_Bias);
    void setInitBodyPosture(const dcm::Vec3 &_Pos, const dcm::Vec3 &_Ang={0,0,0});
    // inline void setInitFootPosture(const dcm::Vec3 &_PosL, const dcm::Vec3 &_PosR, const dcm::Vec3 &_AngL={0,0,0}, const dcm::Vec3 &_AngR={0,0,0});
    inline void setTs(const double &_Ts){this->Ts = _Ts;};
    void setFootSwingHeight(const double &_Height);
    inline void setLogger(blocks::LLog<> *pLogger){this->pExtLogger = pLogger;};
    inline void setSquatTime(const double &_Time){this->Squat.MoveTime = _Time;};
    inline void setGUIFlag(const ljh::tools::GUIStateFlag &_flag){this->GUIFlag = _flag;};
protected: 
    int StateFlag;
    double Time, Ts;
    ljh::tools::GUIStateFlag GUIFlag;
    struct {
        double GoalTime;
        double MoveTime;
    } Squat;
    blocks::LLog<> *pExtLogger;

    // double TarBodyHeight;
    // dcm::Vec3 InitFootPosL, InitFootPosR, InitFootAngL, InitFootAngR, InitBodyPos, InitBodyAng, CoM_Bias;
};
}}