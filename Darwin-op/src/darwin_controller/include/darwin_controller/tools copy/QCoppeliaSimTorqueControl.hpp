#pragma once
#include <iostream>
#include <extApi.h>
#include <extApiPlatform.h>
#include <LCoppeliaSim/src/LCoppeliaSim.hpp>
#include <LCoppeliaSim/LCoppeliaSimBHR8P1.hpp>

using namespace ljh::LCoppeliaSimBHR8P1;
namespace qhx{namespace CoppeliaSim{
    
    template<int CONTROL_TIME_MS=4, int PORT_NUM=19997>
    class QCoppeliaSim:public lee::LCoppeliaSim<Config::_JOINT_NUM, Config::_FS_NUM>
    {
        public:
        void setSingleJointParameter(int joint_seq, int JointParam) //JointParam: 0 Torque Control, 1 Position Control
        {
            simxSetObjectIntParameter(this->ClientID, this->JointHandle[joint_seq], 2001, JointParam, this->ModeRunning);
        }
        void setSingleJointTorque(int joint_seq, double torque)
        {
            this->setSingleJointLimitVelocity(joint_seq, 999.0);
            this->setSingleJointLimitTorque(joint_seq, torque);
        }
        void switchtoTorqueControl(int *joint_param)
        {
            for (int i = 0; i < Config::_JOINT_NUM, i++) 
            {
                if(joint_param[i] == 0) setSingleJointParameter(i, joint_param[i]);
            }
        }
        void setJointTorque(int *joint_param, double *joint_torque)
        {
            for (int i = 0; i < Config::_JOINT_NUM, i++) 
            {
                if(joint_param[i] == 0) joint_setSingleJointTorque(i, joint_torque[i]);
            }
        }
    };
}}
