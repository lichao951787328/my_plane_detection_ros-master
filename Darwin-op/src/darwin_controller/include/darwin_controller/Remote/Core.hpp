// Remote/Core.hpp
// lee, hexb66@bit.edu.cn
// Mar. 19, 2022
#pragma once
#define _REMOTE_BEGIN namespace lee{namespace path{namespace remote{
#define _REMOTE_END }}}
#include <LSimpleRoboticsMath/Biped.hh>
#include <deque>
#include <BHRRobotKinematicParam/BHRRobotParameters.h>
#include <math.h>
using waw::ljh::BHRConfig;

_REMOTE_BEGIN

struct BaseType
{
    double Linear[2], Angular;
    inline BaseType(const double &_vx=0.0, const double &_vy=0.0, const double &_w=0.0)
    {
        Linear[0] = _vx; Linear[1] = _vy; Angular = _w;
    };
};

struct StepType: BaseType
{
    double StepTime;
    inline StepType(const double &_vx=0.0, const double &_vy=0.0, const double &_w=0.0, const double &_step_time = 0.8):BaseType(_vx, _vy, _w), StepTime(_step_time){};
};

using math::biped::FootholdType;
using math::biped::_LEFT__;
using math::biped::_RIGHT_;

template<const int N = 5>
// constexpr int N = 5;
class Remote
{
protected:
    double PathHeight;
    double Ts;
// public:
    FootholdType LastFoothold;
    std::deque<FootholdType> FootholdList;
    std::deque<StepType> StepList;
    // Command velocity: the direct input from user
    BaseType CmdVel;
    // Reference velocity: tracking the CmdVel by PD control, and is used to generate the FootholdList
    BaseType RefVel, dRefVel, ddRefVel;


public:
    inline auto &getCmdVel(){return this->CmdVel;};
    inline auto &getRefVel(){return this->RefVel;};
    inline auto getPlanSize(){return N;};
    inline void setTs(const double& _Ts){this->Ts = _Ts;};
    Remote():PathHeight(0.0),Ts(0.001)
    {
        //Modified For BHR8P1
        double FootY[2] = {BHRConfig.HipWidth/2.0 ,-BHRConfig.HipWidth/2.0 };
        double StepTime = 1.0;
        int FootFlag[2] = {_LEFT__, _RIGHT_};
        for(int i=0;i<N;i++)
        {
            this->StepList.push_back({0,0,0,StepTime});
            this->FootholdList.push_back(FootholdType(FootFlag[i%2], this->StepList[i].StepTime, {0.0, FootY[i%2], 0.0}));
        }
        this->LastFoothold = this->FootholdList[0];
    };

    inline void setCmdVel(const BaseType &_CmdVel){this->CmdVel = _CmdVel;};
    void calRefVel()
    {
        double kp = 10.0, kd = sqrt(2*kp);

        //TODO 修改所有显式调用控制周期进行计算的地方 Ts（包括这个）
        //double Ts = 0.001;
        for(int i=0;i<3;i++)
        {
            this->ddRefVel.Linear[i] = -kp*(this->RefVel.Linear[i] - this->CmdVel.Linear[i]) - kd*this->dRefVel.Linear[i];
            this->dRefVel.Linear[i] += this->ddRefVel.Linear[i] * this->Ts;
            this->RefVel.Linear[i] += this->dRefVel.Linear[i] * this->Ts;
        }
    };
    void calStepholds(const double *_FootPosL, const double *_FootAngL, const double *_FootPosR, const double *_FootAngR)
    {
        auto &vt = this->RefVel.Linear[0], &vr = this->RefVel.Linear[1];
        auto &w = this->RefVel.Angular;
        // When starting from stand, update the stephold according to current foot position 
        if(this->FootholdList.size()==0)
        {
            this->PathHeight = 0.5*(_FootPosL[2]+_FootPosL[2]);
            this->StepList[0].Linear[0] = 0.5*(_FootPosL[0]+_FootPosR[0]);
            this->StepList[0].Linear[1] = 0.5*(_FootPosL[1]+_FootPosR[1]);
            this->StepList[0].Angular   = 0.5*(_FootAngL[2]+_FootAngR[2]);
            this->LastFoothold.FootFlag = math::biped::_LEFT__;
            this->LastFoothold.Pos = math::biped::Vec3{_FootPosL[0],_FootPosL[1],this->PathHeight};
            this->LastFoothold.Ang = math::biped::Vec3{0.0,0.0,_FootAngL[2]};
        }
        // When one step is over, slide the stephold list 
        else if(this->FootholdList.size()<N)
        {
            this->StepList.pop_front();
            this->StepList.resize(N);
        }
        for (int i = 1; i < this->StepList.size(); i++)
        {
            auto T = this->StepList[i-1].StepTime;
            auto &yaw0 = this->StepList[i-1].Angular;
            if(fabs(w)>1e-4)
            {
                auto &yaw  = this->StepList[i].Angular;
                yaw = yaw0 + w*T;
                this->StepList[i].Linear[0] = this->StepList[i-1].Linear[0] + 1/w*vt*sin(yaw) + 1/w*vr*cos(yaw) - 1/w*vt*sin(yaw0) - 1/w*vr*cos(yaw0);
                this->StepList[i].Linear[1] = this->StepList[i-1].Linear[1] + 1/w*vr*sin(yaw) - 1/w*vt*cos(yaw) + 1/w*vt*cos(yaw0) - 1/w*vr*sin(yaw0);
            }
            else            
            {
                this->StepList[i].Linear[0] = this->StepList[i-1].Linear[0] + vt*cos(yaw0) *T - vr*sin(yaw0)*T;
                this->StepList[i].Linear[1] = this->StepList[i-1].Linear[1] + vt*sin(yaw0) *T + vr*cos(yaw0)*T;
                this->StepList[i].Angular   = this->StepList[i-1].Angular;
            }
            this->StepList[i].StepTime = T;
        }
    };
    void calFootholds()
    {
        // Don't change current foothold, which is the target location of swing foot
        if(this->FootholdList.size()==0)
        {
            this->FootholdList.push_back(this->LastFoothold);
        }
        if(this->FootholdList.size()<N) 
        {
            this->FootholdList.resize(N);
            this->LastFoothold = this->FootholdList[0];
        }
        for(int i=1; i<N; i++)
        {
            this->FootholdList[i].FootFlag = math::biped::_RIGHT_ - this->FootholdList[i-1].FootFlag;
            this->FootholdList[i].Ang.setZero();
            if(this->FootholdList[i].FootFlag == math::biped::_RIGHT_)
            {
                this->FootholdList[i].Ang(2) = std::min(this->StepList[i].Angular, this->FootholdList[i-1].Ang(2));
            }
            else
            {
                this->FootholdList[i].Ang(2) = std::max(this->StepList[i].Angular, this->FootholdList[i-1].Ang(2));
            }
            this->calFootholdPos(i);
            this->FootholdList[i].StepTime = this->FootholdList[i-1].StepTime;
        }
    };

    void calFootholdPos(const int &_Index)
    {
        int FootIndex = this->FootholdList[_Index].FootFlag;
        double FlagY = 1.0;
        //Modified For BHR8P1
        double DistY = BHRConfig.HipWidth/2.0;//0.10;//0.08;
        if(FootIndex == math::biped::_RIGHT_) FlagY = -1.0;
        if(fabs(this->RefVel.Angular)>5.0/57.3) DistY =  BHRConfig.HipWidth/2.0 + 0.01;//0.09;
        if(fabs(this->RefVel.Angular)>10.0/57.3) DistY = BHRConfig.HipWidth/2.0 + 0.015;//;0.095;
        if(fabs(this->RefVel.Angular)>15.0/57.3) DistY = BHRConfig.HipWidth/2.0 + 0.02;//0.10;

        double PosY = FlagY*DistY;

        this->FootholdList[_Index].Pos = math::biped::Vec3{this->StepList[_Index].Linear[0],this->StepList[_Index].Linear[1], this->PathHeight};
        math::biped::Vec3 LastHold2Current = this->FootholdList[_Index].getRot().transpose()*(this->FootholdList[_Index-1].Pos - this->FootholdList[_Index].Pos);
        if(FootIndex == math::biped::_LEFT__)
        {
            PosY = std::max(PosY, LastHold2Current(1)+2.0*PosY);
        }
        else
        {
            PosY = std::min(PosY, LastHold2Current(1)+2.0*PosY);
        }
        this->FootholdList[_Index].Pos += this->FootholdList[_Index].getRot()*math::biped::Vec3{0.0, PosY, 0.0};
    };
};

_REMOTE_END