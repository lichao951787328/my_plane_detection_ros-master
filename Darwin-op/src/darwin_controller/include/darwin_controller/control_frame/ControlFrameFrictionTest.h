#pragma once
#include <control_frame/BHR_ControlFrame.h>
#include <tools/StateFlag.h>
#include <tools/JointOrder.hpp>   
#include <cmath>
#include <JointInterpolation/JointInterpolationBlock.h>
#include <tools/StateFlag.h>
#include <vector>
#include <LGui/LGui.h>
_BHR_CF_BEGIN

namespace FrictionTest{
#ifndef PI_ljh
#define PI_ljh 3.1415926535
#endif

enum ControlState
{
    CONTROL_STATE_SECOND_RESET,
    CONTROL_STATE_RUN_SIN_FUNC,
    CONTROL_STATE_THIRD_RESET,
    CONTROL_STATE_NONE
};

enum VELOCITYSTATE
{
    VEL_ZEROFIVE,
    VEL_ONEZERO,
    VEL_ONEFIVE,
    VEL_TWOZERO,
    VEL_TWOFIVE,
    VEL_THREEZERO,
    VEL_THREEFIVE,
    VEL_FOURZERO,
    VEL_FOURFIVE,
    VEL_FIVEZERO,
    VEL_FIVEFIVE,
    VEL_SIXZERO,
    VEL_SIXFIVE,
    VEL_SEVENZERO,
    VEL_SEVENFIVE,
    VEL_EIGHTZERO,
    VEL_EIGHTFIVE,
    VEL_NINEZERO,
    VEL_NINEFIVE,
    VEL_TENZERO,
    VEL_TENFIVE,
};

class ControlFrame:public Frame
{
protected:
    
    ljh::tools::GUIStateFlag GUIFlag;
    

public:
    inline ControlFrame(const double &_Ts):Frame(_Ts){
        
        double SinCycle = 30.0;
        
        this->SinStartAll = false;
        for(int i=0;i<6;i++)
        {
            this->TimeCount[i] = SinCycle * 2; 
            this->SinTime[i] = 0.0;
            this->SinStart[i] = false;
        }
            
        for(int i = 0; i<6; i++)
        {
            // cal the axis of symmetry of sin
            LLegAngleIni[i] = (MaxLLegJointDeg[i] + MinLLegJointDeg[i])/2.0 / 180. * PI_ljh;
            RLegAngleIni[i] = (MaxRLegJointDeg[i] + MinRLegJointDeg[i])/2.0 / 180. * PI_ljh;
            // cal the amp with eps
            LLegAmptitudeSin[i] = ((std::abs(MaxLLegJointDeg[i] - MinLLegJointDeg[i]))/2.0 - this->epsDeg[i]) / 180. * PI_ljh;
            RLegAmptitudeSin[i] = ((std::abs(MaxRLegJointDeg[i] - MinRLegJointDeg[i]))/2.0 - this->epsDeg[i]) / 180. * PI_ljh;
            this->TSin[i] = SinCycle;
        }
        this->iniFlag = true;

        this->VelState = VEL_ZEROFIVE;
       
    };
    inline ControlFrame(){};
    ControlFrame(const ControlFrame & other);
    ControlFrame & operator=(const ControlFrame & other);
    int init();
    int run();
    int log();
    int clear();
    int print();
    inline void setGUIFlag(const ljh::tools::GUIStateFlag &_flag){this->GUIFlag = _flag;};
    
protected:
    double epsDeg[6] = {10., 13., 30., 30., 30., 9.};

    double MinLLegJointDeg[6] = { -16., -20., -90., -2e-2,-70.,-18. };
    double MaxLLegJointDeg[6] = { 24., 32., 28., 135., 50., 18.}; 

    double MinRLegJointDeg[6] = {-24., -32. , -90., -2e-2, -70.,-18.};
    double MaxRLegJointDeg[6] = { 16., 20., 28., 135., 50., 18. };

    double LLegAmptitudeSin[6];
    double RLegAmptitudeSin[6];
    double TSin[6];
    double TimeCount[6];

    double LLegAngleIni[6];
    double RLegAngleIni[6];

    double LLegAngle[6];
    double RLegAngle[6];

    bool SinStart[6];
    bool SinStartAll;

    double SinTime[6];

    int runSin();

    // for interpolation

    double JointRefPos[ljh::ctrl::joint_interpolation::JOINT_NUM];
    double JointRealPos[ljh::ctrl::joint_interpolation::JOINT_NUM];
    double JointRealPosIni[ljh::ctrl::joint_interpolation::JOINT_NUM];

    bool iniFlag;

    void updateJointRefPos();
    void updateJointRealPos();
    void iniJointRefPos();
    

    ControlState CrtlState;
    void runControl();
    void updateJointRealPosIni();
    void loadTargetPosForThirdReset();
    void loadTargetPosForSecondReset();

    // online velocity modification for test convenience
    void modifyVelocityWithKey(int key);
    std::string SpeedtoString();
    VELOCITYSTATE VelState;
    std::vector<VELOCITYSTATE> VelSeries={
        VEL_ZEROFIVE,
        VEL_ONEZERO,
        VEL_ONEFIVE,
        VEL_TWOZERO};
        // VEL_TWOFIVE,
        // VEL_THREEZERO,
        // VEL_THREEFIVE,
        // VEL_FOURZERO,
        // VEL_FOURFIVE,
        // VEL_FIVEZERO};
};

}
_BHR_CF_End