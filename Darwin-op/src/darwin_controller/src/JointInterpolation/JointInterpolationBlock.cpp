#include <JointInterpolation/JointInterpolationBlock.h>

extern "C"{
    #include <LSimpleRoboticsMath/leeMatrix.h>
}

#include <cmath>

_JOINT_INP_BEGIN
int Block::init()
{

    using namespace lee::tools;
    using lee::math::kinematics::ToRad;

    // 设置插值运动时间
    this->MoveTimeS = 2.0;
    
    return 0;
}

int Block::run()
{
    using lee::math::kinematics::ToRad;
    auto &In = this->getInput();
    // 插值过程中，计算插值轨迹
    if(this->IsRunning)
    {
        for(int i=0;i<JOINT_NUM;i++)
        {
            realtime_1D_interpolation_5(
                &this->Joint.Pos(i), 
                &this->Joint.Vel(i), 
                &this->Joint.Acc(i), 
                this->JointTargetPos(i), 
                0.0, 
                0.0, 
                this->Time, 
                this->MoveTimeS, 
                this->Ts
            );
            this->DataOutput.JointRefPos[i] = this->Joint.Pos(i);
        }
        this->Time += this->Ts;
        if(this->Time >= this->MoveTimeS)
        {
            this->IsRunning = false;
            this->Time = 0.0;
            return 1;
        }
    }
    // 非插值过程，按 7 触发插值计算，并以期望关节角度（复位完毕期望与实际相同，为保证空跑正常，设置为期望关节角）为初始状态
    else if(*In.pPressKey == '7')
    {
        this->IsRunning = true;
        for(int i=0;i<JOINT_NUM;i++)
        {
            this->Joint.Pos(i) = this->DataOutput.JointRefPos[i];//  this->DataInput.JointRealPos[i];
        }
    }
    return 0;
}

int Block::print()
{
    if(this->GUIFlag == ljh::tools::GUIStateFlag::GUI_OFF)
    {
        std::cout<<" [JointInterpolation]: ";
        if(this->IsRunning)
            std::cout<< " Running "<<std::endl;
        else
            std::cout<< " NotRunning "<<std::endl;
        
        std::cout<<"InterpolationLeftLegJoint: ";
        for(int i=0;i<6;i++)
        {
            std::cout<<this->Joint.Pos(lee::tools::Left_Leg1 + i) / 3.1415926535 * 180.<<", ";
        }
        std::cout<< std::endl;

        std::cout<<"InterpolationRightLegJoint: ";
        for(int i=0;i<6;i++)
        {
            std::cout<<this->Joint.Pos(lee::tools::RightLeg1 + i) / 3.1415926535 * 180.<<", ";
        }
        std::cout<< std::endl;
    }
    else
    {
        
        // 有待补充
    }
    return 0;
}

int Block::log()
{
    return 0;
}

void Block::setJointTargetPosAll(double *_TargetPos)
{
    for(int i=0; i< JOINT_NUM; i++)
        this->JointTargetPos(i) = _TargetPos[i];
}

void Block::setSingleJointTargetPos(int _JointOrder, const double& _TargetPos)
{
    this->JointTargetPos(_JointOrder) = _TargetPos;
}


_JOINT_INP_END