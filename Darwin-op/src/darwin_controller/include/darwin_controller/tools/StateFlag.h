#pragma once
#include <array>
#include <iostream>
namespace ljh{namespace tools{

enum class GUIStateFlag
{
    GUI_OFF,
    GUI_ON
};

enum ControlType
    {
        StandStill,
        PositionControl,
        TorqueControl,
        PosTorqueMixControl
    };

enum class MotorControlType
{
    MOTOR_CSP,
    MOTOR_CST
};

template<int NUM>
class MotorControlTypeSeries
{
public:
    
    MotorControlTypeSeries()
    {
        // default setting is Position Control for all motors
        this->setAllJointPositionCntl();
        this->RobotCntlType = ControlType::PositionControl;
    }

    MotorControlTypeSeries(MotorControlType _JointCntlType)
    {
        if(_JointCntlType == MotorControlType::MOTOR_CSP)
        {
            this->setAllJointPositionCntl();
            this->RobotCntlType = ControlType::PositionControl;
        }
        else
        {
            this->setAllJointTorqueCntl();
            this->RobotCntlType = ControlType::TorqueControl;
        }
    }
 
    MotorControlTypeSeries(const MotorControlTypeSeries& other)
    {
        for(size_t i=0;i < other.JointCntlType.size();i++)
        {
            this->JointCntlType[i] = other.JointCntlType[i];
        }
        this->RobotCntlType = other.RobotCntlType;  
    }

    MotorControlTypeSeries& operator=(const MotorControlTypeSeries& other)
    {
        if(this != &other)
        {
            for(size_t i = 0;i < other.JointCntlType.size();i++)
            {
                this->JointCntlType[i] = other.JointCntlType[i];
            }
            this->RobotCntlType = other.RobotCntlType;  
        }
        return *this;  
        
    }
    // setters
    void setAllJointTorqueCntl(){this->JointCntlType.fill(MotorControlType::MOTOR_CST);this->RobotCntlType = ControlType::TorqueControl;};
    void setAllJointPositionCntl(){this->JointCntlType.fill(MotorControlType::MOTOR_CSP);this->RobotCntlType = ControlType::PositionControl;};
    void setSingleJointTorqueCntl(int _Index){this->setSingleJointCntlType(MotorControlType::MOTOR_CST, _Index);};
    void setSingleJointPositionCntl(int _Index){this->setSingleJointCntlType(MotorControlType::MOTOR_CSP, _Index);};
    void setSingleJointCntlType(MotorControlType _CntlType, int _Index){JointCntlType[_Index] = _CntlType;};
    
    // getters
    std::array<MotorControlType, NUM> & getJointCntlTypeArray(){return this->JointCntlType;};
    MotorControlType * getJointCntlTypePtr(){return this->JointCntlType.data();};
    MotorControlType getSingleJointCntlType(int _Index){return this->JointCntlType[_Index];};
    
    // checkers
    ControlType checkAndGetRobotCntlType()
    {
        bool PosCntlFlag = true;
        bool TorCntlFlag = true;
        for( auto & i:this->JointCntlType)
        {
            if(i==MotorControlType::MOTOR_CSP)
                TorCntlFlag = false;
            else if(i==MotorControlType::MOTOR_CST)
                PosCntlFlag = false;
            else
                std::cerr<<"Motor Control Type is wrong!"<<std::endl;
        }

        if(PosCntlFlag == true)
            this->RobotCntlType = ControlType::PositionControl;
        else if(TorCntlFlag == true)
            this->RobotCntlType = ControlType::TorqueControl;  
        else
            this->RobotCntlType = ControlType::PosTorqueMixControl; 

        return this->RobotCntlType;  
    }
private:
    std::array<MotorControlType, NUM> JointCntlType;
    ControlType RobotCntlType;

};

}}

