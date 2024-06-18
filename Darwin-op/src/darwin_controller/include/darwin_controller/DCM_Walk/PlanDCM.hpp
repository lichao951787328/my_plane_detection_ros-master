//DCM_Walk/PlanDCM.hpp
#pragma once
#include "Base.h"
#include <deque>
#include <math.h>
_DCM_BEGIN

class StepDCM
{
public:
    inline StepDCM(const double &_Zc=0.8){
        this->setZc(_Zc);
        this->eCMP_EndInFoot = Vec3{0.0, 0, 0};
        this->eCMP_IniInFoot = Vec3{0.0, 0, 0};
    };
    inline void setZc(const double &_Zc){this->Zc = _Zc; this->B = calDCM_ConstantB(_Zc);};
    inline void updateStepTime(const double &_StepTime, const double &_DS_Ratio=0.2){
        this->StepTime = _StepTime;
        this->DS_Ratio = _DS_Ratio;
        this->StepTimeDS = this->StepTime*_DS_Ratio;
        this->StepTimeSS = this->StepTime - this->StepTimeDS;
    };
    inline void calDCM_End(const Vec3 &_NextStepDCM_Ini, const double &_NextStepTimeSS, const double &_NextStepTimeDS){
        double new_step_time = 0.5*_NextStepTimeDS + _NextStepTimeSS + 0.5*this->StepTimeDS;
        // this->DCM_End = calDCM_Pos(this->VRP_End, _NextStepDCM_Ini, -new_step_time*0.5, this->B);
        this->DCM_End = calDCM_Pos(this->VRP_End, _NextStepDCM_Ini, -new_step_time, this->B);
    };
    inline void calDCM_Ini(const double &_NextStepTimeSS, const double &_NextStepTimeDS){
        double new_step_time = 0.5*_NextStepTimeDS + _NextStepTimeSS + 0.5*this->StepTimeDS;
        // this->DCM_Ini = calDCM_Pos(this->VRP_Ini, this->DCM_End, -new_step_time*0.5, this->B);
        this->DCM_Ini = this->DCM_End;
    };
    inline void calDCM_DS_Ini(const Vec3 &_LastStepVRP_End){
        this->DCM_DS_Ini.Pos = calDCM_Pos(_LastStepVRP_End, this->DCM_Ini, -this->StepTimeDS*0.5, this->B);
        this->DCM_DS_Ini.Vel = calDCM_Vel(_LastStepVRP_End, this->DCM_Ini, -this->StepTimeDS*0.5, this->B);
        this->DCM_DS_Ini.Acc = calDCM_Acc(_LastStepVRP_End, this->DCM_Ini, -this->StepTimeDS*0.5, this->B);
    };
    inline void calDCM_DS_End(){
        this->DCM_DS_End.Pos = calDCM_Pos(this->VRP_Ini, this->DCM_Ini, this->StepTimeDS*0.5, this->B);
        this->DCM_DS_End.Vel = calDCM_Vel(this->VRP_Ini, this->DCM_Ini, this->StepTimeDS*0.5, this->B);
        this->DCM_DS_End.Acc = calDCM_Acc(this->VRP_Ini, this->DCM_Ini, this->StepTimeDS*0.5, this->B);
    };
    inline void calVRP_IniAndEndPos(const FootholdType &_FootHold){
        this->VRP = _FootHold.Pos + Vec3{0,0,this->Zc};
        auto foot_rot = _FootHold.getRot();
        this->VRP_Ini = _FootHold.Pos + foot_rot*this->eCMP_IniInFoot + Vec3{0,0,this->Zc};
        this->VRP_End = _FootHold.Pos + foot_rot*this->eCMP_EndInFoot + Vec3{0,0,this->Zc};
    };
// protected:
    double StepTime, DS_Ratio, StepTimeDS, StepTimeSS, B, Zc;
    Vec3 VRP_Ini, VRP_End, DCM_Ini, DCM_End, VRP;
    State3 DCM_DS_Ini, DCM_DS_End;

    Vec3 eCMP_IniInFoot, eCMP_EndInFoot;
};

template<const int N=5>
// constexpr int N=5;
class DCM_Planner
{
protected:
    double Zc, B;
    StepDCM LastStep;
    StepDCM StepList[N];
    State3 DCM, VRP, CoM;
    Vec3 DCM_InitPos;
    
    // The following members are moved outside
    // int WalkState;
    // double TimeInStep;

public:
    // Construction, default CoM height is 0.8m, and default sample time or the control period is 5ms
    DCM_Planner(const double _Zc=0.8) :
        Zc(_Zc), B(calDCM_ConstantB(_Zc))
    {
        for(int i=0;i<N;i++) this->StepList[i].setZc(_Zc);
    };

    // Get the number of plan steps
    inline int getPlanStepNum(){return N;};
    
    // Initialize with given CoM postion and velocity. The velocity is set to zero by default. The DCM position is calculated by CoM state, and the velocity is zero.
    void init(const Vec3 &_CoM_Pos, const Vec3 &_CoM_Vel=Vec3::Zero()){
        this->CoM.Pos = _CoM_Pos;
        this->CoM.Vel = _CoM_Vel;
        this->CoM.Acc.setZero();
        this->DCM.Pos = _CoM_Pos + this->B*_CoM_Vel;
        this->DCM.Vel.setZero();    this->DCM.Acc.setZero();
        this->DCM_InitPos = this->DCM.Pos; 
    };
    
    // Calculate VRP postions and time information of each step
    void calVRP_AndTime(const std::deque<FootholdType> &_FootholdList, const int &_WalkState)
    {
        int remain_step_num = std::min((int)_FootholdList.size(), N);
        if(remain_step_num==0) return;

        for(int i=0;i<remain_step_num;i++)
        {
            this->StepList[i].calVRP_IniAndEndPos(_FootholdList[i]);
            this->StepList[i].updateStepTime(_FootholdList[i].StepTime);
        }
        if(_WalkState == WALK_INIT)
        {
            // this->StepList[0].VRP_Ini = this->DCM_InitPos;
            this->StepList[0].updateStepTime(this->StepList[0].StepTime, 1.0);
            this->LastStep = this->StepList[0];
        }
    };
    
    // Calculate key points of DCM in each step
    void calDCM_KeyPoints(const std::deque<FootholdType> &_FootholdList){
        int remain_step_num = std::min((int)_FootholdList.size(), N);
        if(remain_step_num==0) return;

        int end = remain_step_num-1;
        auto &EndStep = this->StepList[end];

        // Get the end of N plan steps
        StepDCM *pLastStep;
        // Current step is the final step
        if(end==0)
        {
            pLastStep = &this->LastStep;
        }
        // Current step is not the final step
        else
        {
            pLastStep = &this->StepList[end-1];
        }
        EndStep.VRP_End = 0.5*(EndStep.VRP+pLastStep->VRP); 
        EndStep.VRP_Ini = EndStep.VRP_End;
        EndStep.DCM_End = EndStep.VRP_End;
        EndStep.DCM_DS_End.Pos = EndStep.VRP_End;
        EndStep.DCM_DS_End.Vel.setZero();
        EndStep.DCM_DS_End.Acc.setZero();
        EndStep.calDCM_Ini(EndStep.StepTimeSS, EndStep.StepTimeDS);
        EndStep.calDCM_DS_Ini(pLastStep->VRP_End);
            
        // If current step is the final step, then no need to calculate other steps
        if(end==0) return;
        
        // Get other steps
        for(int i=end-1;i>=0;i--)
        {
            if(i==0) 
                pLastStep = &this->LastStep;
            else   
                pLastStep = &this->StepList[i-1];
            this->StepList[i].calDCM_End(this->StepList[i+1].DCM_Ini,this->StepList[i+1].StepTimeSS, this->StepList[i+1].StepTimeDS);
            this->StepList[i].calDCM_Ini(this->StepList[i+1].StepTimeSS, this->StepList[i+1].StepTimeDS);
            this->StepList[i].calDCM_DS_Ini(pLastStep->VRP_End);
            this->StepList[i].calDCM_DS_End();
        }
    };
    
    // Calculate DCM trajectory according to key points. If current step is over, return STEP_OVER; otherwise return STEP_CONTINUE
    int calDCM_Trajectory(const double &_TimeInStep, const int &_WalkState, const double &_Ts){
        auto &step0 = this->StepList[0];
        // The initialize step only has the DS phase: to DCM_DS_End directly
        if(_WalkState == WALK_INIT)
        {
            calRealTimeInterpolation(this->DCM, step0.DCM_DS_End,_TimeInStep, step0.StepTime, _Ts);
            if(_TimeInStep > step0.StepTime - 1.1*_Ts) return STEP_OVER; else return STEP_CONTINUE;
        }
        
        // Single support phase: to DCM_DS_Ini
        if(_TimeInStep < step0.StepTimeSS)
        {
            calRealTimeInterpolation(this->DCM, step0.DCM_DS_Ini,_TimeInStep, step0.StepTimeSS, _Ts);
        }            
        // Double support phase: to DCM_DS_End
        else
        {
            calRealTimeInterpolation(this->DCM, step0.DCM_DS_End,_TimeInStep, step0.StepTime, _Ts);
        }
        if(_TimeInStep > step0.StepTime - 1.1*_Ts) return STEP_OVER; else return STEP_CONTINUE;
    };
    
    inline void calCoM_Trajectory(const double &_Ts){
        this->CoM.Vel = -1./this->B*(this->CoM.Pos-this->DCM.Pos);
        this->CoM.Pos = this->CoM.Pos+this->CoM.Vel*_Ts;
        this->CoM.Acc = -1./this->B*(this->CoM.Vel-this->DCM.Vel);
    };
    inline void calVRP_Trajectory(){
        this->VRP.Pos = this->DCM.Pos - this->B*this->DCM.Vel;
    };
};

_DCM_END