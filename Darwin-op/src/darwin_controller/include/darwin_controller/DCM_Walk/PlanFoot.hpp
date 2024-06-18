// DCM_Walk/PlanFoot.hpp 
// lee, hexb66@bit.edu.cn
// Mar. 13, 2022
#pragma once
#include "Base.h"
#include <BHRRobotKinematicParam/BHRRobotParameters.h>
#include <math.h>
using waw::ljh::BHRConfig;

_DCM_BEGIN
struct FootState{State3 Linear, Angular;};
class FootPlanner
{
protected:
    int Index;
    double FootSwingHeight;
    FootState Foot[2];
    FootState FootTarget[2];
    FootState LastFootTarget[2];
    
public:
    FootPlanner():Index(_LEFT__)
    {
        this->FootSwingHeight = 0.05;
    };
    void setFootSwingHeight(const double &Height){this->FootSwingHeight = Height;};
    void init(const Vec3 &_FootPosL, const Vec3 &_FootPosR, const Vec3 &_FootAngL=Vec3::Zero(), const Vec3 &_FootAngR=Vec3::Zero())
    {
        this->Foot[_LEFT__].Linear.Pos  = _FootPosL;
        this->Foot[_RIGHT_].Linear.Pos  = _FootPosR;
        this->Foot[_LEFT__].Angular.Pos = _FootAngL;
        this->Foot[_RIGHT_].Angular.Pos = _FootAngR;
        this->FootTarget[_LEFT__] = this->Foot[_LEFT__];
        this->FootTarget[_RIGHT_] = this->Foot[_RIGHT_];
        this->LastFootTarget[_LEFT__] = this->Foot[_LEFT__];
        this->LastFootTarget[_RIGHT_] = this->Foot[_RIGHT_];
    };
    void updateFootTarget(const FootholdType &_Foothold)
    {
        this->Index = _Foothold.FootFlag;
        this->FootTarget[this->Index].Angular.Pos = _Foothold.Ang;
        this->FootTarget[this->Index].Linear.Pos  = _Foothold.Pos;
    };
    void calFootTrajectory(const double &_TimeInStep, const double &_StepTimeSS, const double &_StepTime, const double &_Ts)
    {
        int _SupID = math::biped::_RIGHT_ - this->Index;

        auto _Tar = this->FootTarget[this->Index].Linear;
        double _MaxH = std::max(_Tar.Pos(2), this->LastFootTarget[this->Index].Linear.Pos(2));
        _MaxH = std::max(_MaxH, this->FootTarget[_SupID].Linear.Pos(2));
        
        // Up
        if(this->LastFootTarget[this->Index].Linear.Pos(2)+0.005 < _MaxH)
        {
            this->calFootTraUp(_Tar, _MaxH, _TimeInStep, _StepTimeSS, _StepTime, _Ts);
        }
        else if(this->FootTarget[this->Index].Linear.Pos(2)+0.005 < _MaxH)
        {
            this->calFootTraDown(_Tar, _MaxH, _TimeInStep, _StepTimeSS, _StepTime,  _Ts);
        }
        else
        {
            this->calFootTraNormal(_Tar, _MaxH, _TimeInStep, _StepTimeSS, _StepTime,  _Ts);
        }

    };
protected:
    // Walking on even terrain
    void calFootTraNormal(State3 &_Tar, const double &_MaxHeight, const double &_TimeInStep, const double &_StepTimeSS, const double &_StepTime, const double &_Ts)
    {
        int sup_id = math::biped::_RIGHT_ - this->Index;

        if(_TimeInStep <= _StepTimeSS)
        {
            // x, y and rotation
            calRealTimeInterpolation<3, 0>(this->Foot[this->Index].Linear, this->FootTarget[this->Index].Linear, _TimeInStep, _StepTimeSS, _Ts);
            calRealTimeInterpolation<3, 1>(this->Foot[this->Index].Linear, this->FootTarget[this->Index].Linear, _TimeInStep, _StepTimeSS, _Ts);
            calRealTimeInterpolation(this->Foot[this->Index].Angular, this->FootTarget[this->Index].Angular, _TimeInStep, _StepTimeSS, _Ts);
            // z
            if (_TimeInStep < 0.5 * _StepTimeSS)
            {
                _Tar.Pos(2) = _MaxHeight + this->FootSwingHeight;
                calRealTimeInterpolation<3, 2>(this->Foot[this->Index].Linear, _Tar, _TimeInStep, 0.5 * _StepTimeSS, _Ts);
            }
            else
            {
                _Tar.Pos(2) += 0.0015;
                calRealTimeInterpolation<3, 2>(this->Foot[this->Index].Linear, _Tar, _TimeInStep, _StepTimeSS, _Ts);
            }
        }
        else
        {
            // Z 
            calRealTimeInterpolation<3,2>(this->Foot[this->Index].Linear, this->FootTarget[Index].Linear, _TimeInStep, _StepTime, _Ts);
            // Pitch
            calRealTimeInterpolation<3,1>(this->Foot[this->Index].Angular, this->FootTarget[Index].Angular, _TimeInStep, _StepTime, _Ts);
        }
        
    };

    // Walking on uneven terrain with toe-off and heel-down 
    void calFootTraNormalToeOff(State3 &_Tar, const double &_MaxHeight, const double &_TimeInStep, const double &_StepTimeSS, const double &_StepTime, const double &_Ts)
    {
        double SwingDownDeg = -5.0;
        double SupDeg = 0.0;
        double FootLenFront = BHRConfig.FootFord;// 0.154;
        double FootLenBack = BHRConfig.FootBack;// 0.071;
        int sup_id = math::biped::_RIGHT_ - this->Index;

        // Single Support Phase
        if (_TimeInStep <= _StepTimeSS)
        {
            double b = 0.5;
            auto _TarAng = this->FootTarget[Index].Angular;

            // Swing up 
            if (_TimeInStep < b * _StepTimeSS)
            {
                _Tar.Pos(2) = _MaxHeight + this->FootSwingHeight;
                calRealTimeInterpolation<3, 2>(this->Foot[this->Index].Linear, _Tar, _TimeInStep, b* _StepTimeSS, _Ts);
                calRealTimeInterpolation<3, 0>(this->Foot[this->Index].Linear, _Tar, _TimeInStep, _StepTimeSS, _Ts);
                calRealTimeInterpolation<3, 1>(this->Foot[this->Index].Linear, _Tar, _TimeInStep, _StepTimeSS, _Ts);
                calRealTimeInterpolation<3>(this->Foot[this->Index].Angular, _TarAng, _TimeInStep, b * _StepTimeSS, _Ts);
            }
            // Swing down 
            else
            {
                _TarAng.Pos(1) +=SwingDownDeg*math::kinematics::ToRad;
                _Tar.Pos(0) -= FootLenBack*(1.0-cos(-_TarAng.Pos(1)));
                _Tar.Pos(2) += (0.0015 + FootLenBack*sin(-_TarAng.Pos(1)));
                calRealTimeInterpolation<3>(this->Foot[this->Index].Linear,  _Tar,    _TimeInStep, _StepTimeSS, _Ts);
                calRealTimeInterpolation<3>(this->Foot[this->Index].Angular, _TarAng, _TimeInStep, _StepTimeSS, _Ts);
            }
        }
        // Double Support Phase
        else
        {
            // Swing foot: 
            // Z 
            calRealTimeInterpolation<3,2>(this->Foot[this->Index].Linear, this->FootTarget[Index].Linear, _TimeInStep, _StepTime, _Ts);
            // Pitch
            calRealTimeInterpolation<3,1>(this->Foot[this->Index].Angular, this->FootTarget[Index].Angular, _TimeInStep, _StepTime, _Ts);
            // Support foot:  
            auto _TarSup = this->FootTarget[sup_id];
            _TarSup.Angular.Pos(1) += SupDeg*math::kinematics::ToRad;
            _TarSup.Linear.Pos(0)  += FootLenFront*(1.0-cos(_TarSup.Angular.Pos(1)));
            _TarSup.Linear.Pos(2)  += FootLenFront*sin(_TarSup.Angular.Pos(1));
            calRealTimeInterpolation<3>(this->Foot[sup_id].Linear,  _TarSup.Linear,  _TimeInStep, _StepTime, _Ts);
            calRealTimeInterpolation<3>(this->Foot[sup_id].Angular, _TarSup.Angular, _TimeInStep, _StepTime, _Ts);
        }
    };

    // Walking up stages 
    void calFootTraUp(State3 &_Tar, const double &_MaxHeight, const double &_TimeInStep, const double &_StepTimeSS, const double &_StepTime, const double &_Ts)
    {
        double SwingUpFastDeg = 3.0;
        double SwingDownDeg = -10.0;
        double SupDeg = 0.0;
        double FootLenFront = 0.128;
        double FootLenBack = 0.110;
        double SwingDownContactHeight = 0.005;

        int sup_id = math::biped::_RIGHT_ - this->Index;

        // Single Support Phase
        if (_TimeInStep <= _StepTimeSS)
        {
            double a = 0.3;
            double b = 0.5;
            auto _TarAng = this->FootTarget[Index].Angular;

            // Swing up fast 
            if (_TimeInStep < a * _StepTimeSS)
            {
                double _L = FootLenFront;
                _TarAng.Pos(1) += SwingUpFastDeg*math::kinematics::ToRad;
                _Tar.Pos(0) = this->LastFootTarget[Index].Linear.Pos(0)+(_L*(1.0-cos(_TarAng.Pos(1)))-0.02)*cos(this->LastFootTarget[Index].Angular.Pos(2));
                _Tar.Pos(1) = this->LastFootTarget[Index].Linear.Pos(1)+(_L*(1.0-cos(_TarAng.Pos(1)))-0.02)*sin(this->LastFootTarget[Index].Angular.Pos(2));
                _Tar.Pos(2) = _MaxHeight + 0.005 + _L*sin(_TarAng.Pos(1));
                calRealTimeInterpolation<3>(this->Foot[this->Index].Linear,  _Tar,    _TimeInStep, a * _StepTimeSS, _Ts);
                calRealTimeInterpolation<3>(this->Foot[this->Index].Angular, _TarAng, _TimeInStep, a * _StepTimeSS, _Ts);
            }
            // Swing up 
            else if (_TimeInStep < b * _StepTimeSS)
            {
                _Tar.Pos(2) = _MaxHeight +0.02;
                calRealTimeInterpolation<3, 2>(this->Foot[this->Index].Linear, _Tar, _TimeInStep, b* _StepTimeSS, _Ts);
                calRealTimeInterpolation<3, 0>(this->Foot[this->Index].Linear, _Tar, _TimeInStep, _StepTimeSS, _Ts);
                calRealTimeInterpolation<3, 1>(this->Foot[this->Index].Linear, _Tar, _TimeInStep, _StepTimeSS, _Ts);
                calRealTimeInterpolation<3>(this->Foot[this->Index].Angular, _TarAng, _TimeInStep, b * _StepTimeSS, _Ts);
            }
            // Swing down 
            else
            {
                _TarAng.Pos(1) +=SwingDownDeg*math::kinematics::ToRad;
                _Tar.Pos(0) -= FootLenBack*(1.0-cos(-_TarAng.Pos(1)));
                _Tar.Pos(2) += (SwingDownContactHeight + FootLenBack*sin(-_TarAng.Pos(1)));
                calRealTimeInterpolation<3>(this->Foot[this->Index].Linear,  _Tar,    _TimeInStep, _StepTimeSS, _Ts);
                calRealTimeInterpolation<3>(this->Foot[this->Index].Angular, _TarAng, _TimeInStep, _StepTimeSS, _Ts);
            }
        }
        // Double Support Phase
        else
        {
            // Swing foot: 
            // Z 
            calRealTimeInterpolation<3,2>(this->Foot[this->Index].Linear, this->FootTarget[Index].Linear, _TimeInStep, _StepTime, _Ts);
            // Pitch
            calRealTimeInterpolation<3,1>(this->Foot[this->Index].Angular, this->FootTarget[Index].Angular, _TimeInStep, _StepTime, _Ts);
            // Support foot:  
            auto _TarSup = this->FootTarget[sup_id];
            _TarSup.Angular.Pos(1) += SupDeg*math::kinematics::ToRad;
            _TarSup.Linear.Pos(0)  += FootLenFront*(1.0-cos(_TarSup.Angular.Pos(1)));
            _TarSup.Linear.Pos(2)  += FootLenFront*sin(_TarSup.Angular.Pos(1));
            calRealTimeInterpolation<3>(this->Foot[sup_id].Linear,  _TarSup.Linear,  _TimeInStep, _StepTime, _Ts);
            calRealTimeInterpolation<3>(this->Foot[sup_id].Angular, _TarSup.Angular, _TimeInStep, _StepTime, _Ts);
        }
    };

    void calFootTraDown(State3 &_Tar, const double &_MaxHeight, const double &_TimeInStep, const double &_StepTimeSS, const double &_StepTime, const double &_Ts)
    {
        int sup_id = math::biped::_RIGHT_ - this->Index;
        // Single Support Phase
        if (_TimeInStep <= _StepTimeSS)
        {
            // x, y and rotation
            double a = 0.6;
            calRealTimeInterpolation<3, 0>(this->Foot[this->Index].Linear, this->FootTarget[this->Index].Linear, _TimeInStep, a*_StepTimeSS, _Ts);
            calRealTimeInterpolation<3, 1>(this->Foot[this->Index].Linear, this->FootTarget[this->Index].Linear, _TimeInStep, a*_StepTimeSS, _Ts);
            calRealTimeInterpolation(this->Foot[this->Index].Angular, this->FootTarget[this->Index].Angular, _TimeInStep, a*_StepTimeSS, _Ts);
            
            // z
            if (_TimeInStep < a * _StepTimeSS)
            {
                _Tar.Pos(2) = _MaxHeight + 0.02;
                calRealTimeInterpolation<3, 2>(this->Foot[this->Index].Linear, _Tar, _TimeInStep, a * _StepTimeSS, _Ts);
            }
            else
            {
                _Tar.Pos(2) += 0.0015;
                calRealTimeInterpolation<3, 2>(this->Foot[this->Index].Linear, _Tar, _TimeInStep, _StepTimeSS, _Ts);
            }
        }
        // Double Support Phase
        else
        {
            // Z 
            calRealTimeInterpolation<3,2>(this->Foot[this->Index].Linear, this->FootTarget[Index].Linear, _TimeInStep, _StepTime, _Ts);
            // Pitch
            calRealTimeInterpolation<3,1>(this->Foot[this->Index].Angular, this->FootTarget[Index].Angular, _TimeInStep, _StepTime, _Ts);
        }
    };
};

_DCM_END