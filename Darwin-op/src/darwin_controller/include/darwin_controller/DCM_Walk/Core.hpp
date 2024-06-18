// DCM_Walk/Core.hpp
// 6-D walking generation including 3-D positions and 3-D orientations based on Divergent Component of Motion
// lee <hexb66@bit.edu.cn>
#pragma once
#include "Base.h"
#include "PlanDCM.hpp"
#include "PlanFoot.hpp"
#include "PlanBody.hpp"
_DCM_BEGIN
// N is plan step number of one calculation. The final step is treated as the stop step.
template<const int N=5>
class DCM_Walk: public DCM_Planner<N>, public FootPlanner, public BodyPlanner
{
protected:
public:
    int WalkState;
    int StepState;
    int CurrentStep;
    double TimeInStep;
    double Ts;
    // std::deque<FootholdType> FootholdList;
    std::deque<FootholdType> *pFootholdList;

public:
    DCM_Walk(const double &_Zc=0.8, const double &_Ts=0.005)
    :
    DCM_Planner<N>(_Zc), FootPlanner(), BodyPlanner(),
    Ts(_Ts), CurrentStep(0), TimeInStep(0),
    WalkState(STAND), StepState(STEP_CONTINUE)
    {};

    void initPlanner(const Vec3 &_CoM_Pos, const Vec3 &_FootPosL, const Vec3 &_FootPosR, const Vec3 &_FootAngL=Vec3::Zero(), const Vec3 &_FootAngR=Vec3::Zero())
    {
        this->TimeInStep = 0.0;
        this->CurrentStep = 0;
        this->WalkState = STAND;
        this->StepState = STEP_CONTINUE;
        static_cast<DCM_Planner<N>*>(this)->init(_CoM_Pos);
        static_cast<FootPlanner   *>(this)->init(_FootPosL, _FootPosR, _FootAngL, _FootAngR);
    };

    inline void startWalk()
    {
        if(this->WalkState == STAND)
            this->WalkState = WALK_INIT;
    };

    inline void stopWalk()
    {
        if(this->getFootholdList().size()==0) return;
        lee::dcm::StepDCM *p = this->StepList;
        if(this->WalkState == WALK_SS)
            this->getFootholdList().resize(1);
        auto final_foot_hold = this->getFootholdList()[0];
        int index = _RIGHT_-final_foot_hold.FootFlag;
        final_foot_hold.FootFlag = index;
        final_foot_hold.Pos = this->FootTarget[index].Linear.Pos;
        final_foot_hold.Ang = this->FootTarget[index].Angular.Pos;
        this->getFootholdList().push_back(final_foot_hold);
    };

    void plan()
    {
        std::cout<<"dcm plan:";
        std::cout<<this->getFootholdList().size()<<std::endl;
        if(this->getFootholdList().size()==0) this->WalkState = STAND;
        std::cout<<"this->WalkState: "<<this->WalkState<<std::endl;
        switch (this->WalkState)
        {
        case STAND:
            this->calVRP_Trajectory();
            this->calCoM_Trajectory(this->Ts);
            break;
        case WALK_INIT:
        case WALK_SS:
        case WALK_DS:
            this->planDCM();
            this->planFoot();
            this->calBodyAng(this->Foot[0].Angular, this->Foot[1].Angular);
            this->updateStep();
        default:
            break;
        }
    };

    void planDCM()
    {
        this->calVRP_AndTime(this->getFootholdList(), this->WalkState);
        this->calDCM_KeyPoints(this->getFootholdList());
        this->StepState = this->calDCM_Trajectory(this->TimeInStep, this->WalkState, this->Ts);
        this->calCoM_Trajectory(this->Ts);
        this->calVRP_Trajectory();
    };

    void planFoot()
    {
        if(this->WalkState!=WALK_INIT)
        {
            this->updateFootTarget(this->getFootholdList()[0]);
            this->calFootTrajectory(this->TimeInStep, this->StepList[0].StepTimeSS, this->StepList[0].StepTime, this->Ts);
        }
    };

    void updateStep()
    {
        if(this->WalkState == WALK_SS && this->TimeInStep >= this->StepList[0].StepTimeSS)
        {
            this->WalkState = WALK_DS;
        }
        if(this->StepState==STEP_OVER)
        {
            this->TimeInStep = 0.0;
            this->WalkState = WALK_SS;
            this->getFootholdList().pop_front();
            if(this->getFootholdList().size()==0)
            {
                this->WalkState = STAND;
                return;
            }
            this->LastStep = this->StepList[0];
            this->CurrentStep++;

            this->LastFootTarget[0] = this->FootTarget[0];
            this->LastFootTarget[1] = this->FootTarget[1];
        }
        this->TimeInStep+=this->Ts;
    };

    inline auto setFootholdList (std::deque<FootholdType> * p){this->pFootholdList = p;};
    inline auto &getStepList    (){return this->StepList;       };
    inline auto &getFootholdList(){return *this->pFootholdList; };
    inline auto &getDCM         (){return this->DCM;            };
    inline auto &getVRP         (){return this->VRP;            };
    inline auto &getCoM         (){return this->CoM;            };
    inline auto &getWalkState   (){return this->WalkState;      };
    inline auto &getFoot(const int &_ID){
        return this->Foot[_ID];
    };
    inline int getSupFlag      (){
        if(this->WalkState == STAND || this->WalkState == WALK_DS || this->WalkState == WALK_INIT)
            return DS_SUP;
        if(this->WalkState == WALK_SS)
            return _RIGHT_ - this->getFootholdList()[0].FootFlag;
        else
            return DS_SUP;
    }
};
_DCM_END