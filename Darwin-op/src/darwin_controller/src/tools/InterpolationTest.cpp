#include <tools/InterpolationTest.h>

#include <BHRRobotKinematicParam/BHRRobotParameters.h>
#include <cmath>
_INTERP_TEST_BEGIN
int Block::init()
{
    using waw::ljh::BHRConfig;
    using lee::math::kinematics::ToRad;
    // 设置GUI显示Flag
    this->GUIFlag = ljh::tools::GUIStateFlag::GUI_OFF;
    // 设置插值运动时间  
    this->MoveTimeS = 2.0;
    this->SumMoveTimes = this->MoveTimeS;
    // 设置一次按键对应上下蹲起运动总次数
    this->DunCountMax = 10;
    // 计算质心在踝关节连线正上方UP&DOWN
    const double JointPosUp5 = this->Joint5UpDeg * ToRad;
    const double JointPosUp3 = std::asin(BHRConfig.CrusLen * std::sin(JointPosUp5) / BHRConfig.ThighLen);
    const double JointPosUp4 = -(JointPosUp3 + JointPosUp5);

    const double JointPosDown5 = this->Joint5DownDeg * ToRad;
    const double JointPosDown3 = std::asin(BHRConfig.CrusLen * std::sin(JointPosDown5) / BHRConfig.ThighLen);
    const double JointPosDown4 = -(JointPosDown3 + JointPosDown5);

    // 四肢关节角数组赋初值
    this->LArmJointTargetPosUp.setZero();
    this->RArmJointTargetPosUp.setZero();
    this->LLegJointTargetPosUp.setZero();
    this->RLegJointTargetPosUp.setZero();

    this->LArmJointTargetPosDown.setZero();
    this->RArmJointTargetPosDown.setZero();
    this->LLegJointTargetPosDown.setZero();
    this->RLegJointTargetPosDown.setZero();

    this->LLegJointTargetPosUp(2) = JointPosUp3;
    this->LLegJointTargetPosUp(3) = JointPosUp4;
    this->LLegJointTargetPosUp(4) = JointPosUp5;

    this->RLegJointTargetPosUp(2) = JointPosUp3;
    this->RLegJointTargetPosUp(3) = JointPosUp4;
    this->RLegJointTargetPosUp(4) = JointPosUp5;

    this->LLegJointTargetPosDown(2) = JointPosDown3;
    this->LLegJointTargetPosDown(3) = JointPosDown4;
    this->LLegJointTargetPosDown(4) = JointPosDown5;

    this->RLegJointTargetPosDown(2) = JointPosDown3;
    this->RLegJointTargetPosDown(3) = JointPosDown4;
    this->RLegJointTargetPosDown(4) = JointPosDown5;

    return 0;
}

void Block::InerpolationLimbJoint(lee::math::kinematics::Vec<LIMB_NUM> JointTargetPos, enum LimbState LimbFlag)
{
    switch (LimbFlag)
    {
    case LArm:
        for(int i=0;i<LIMB_NUM;i++)
        {
         realtime_1D_interpolation_5(
                     &this->LArmJoint.Pos(i), 
                     &this->LArmJoint.Vel(i), 
                     &this->LArmJoint.Acc(i), 
                     JointTargetPos(i), 
                     0.0, 
                     0.0, 
                     this->Time, 
                     this->SumMoveTimes, 
                     this->Ts
         );
         this->DataOutput.LArmJointRefPos[i] = this->LArmJoint.Pos(i);
        }

        break;

    case RArm:
        for(int i=0;i<LIMB_NUM;i++)
        {
         realtime_1D_interpolation_5(
                     &this->RArmJoint.Pos(i), 
                     &this->RArmJoint.Vel(i), 
                     &this->RArmJoint.Acc(i), 
                     JointTargetPos(i), 
                     0.0, 
                     0.0, 
                     this->Time, 
                     this->SumMoveTimes, 
                     this->Ts
         );
         this->DataOutput.RArmJointRefPos[i] = this->RArmJoint.Pos(i);
        }
        
        break;
    
    case LLeg:
        for(int i=0;i<LIMB_NUM;i++)
        {
         realtime_1D_interpolation_5(
                     &this->LLegJoint.Pos(i), 
                     &this->LLegJoint.Vel(i), 
                     &this->LLegJoint.Acc(i), 
                     JointTargetPos(i), 
                     0.0, 
                     0.0, 
                     this->Time, 
                     this->SumMoveTimes, 
                     this->Ts
         );
         this->DataOutput.LLegJointRefPos[i] = this->LLegJoint.Pos(i);
        }
        
        break;

    case RLeg:
        for(int i=0;i<LIMB_NUM;i++)
        {
         realtime_1D_interpolation_5(
                     &this->RLegJoint.Pos(i), 
                     &this->RLegJoint.Vel(i), 
                     &this->RLegJoint.Acc(i), 
                     JointTargetPos(i), 
                     0.0, 
                     0.0, 
                     this->Time, 
                     this->SumMoveTimes, 
                     this->Ts
         );
         this->DataOutput.RLegJointRefPos[i] = this->RLegJoint.Pos(i);
        }
        
        break;
    default:
        break;
    }
}

int Block::run()
{
    using lee::math::kinematics::ToRad;
    auto &In = this->getInput();
    // 插值过程中，计算插值轨迹
    if(this->IsRunning)
    {
        switch (this->DunStateFlag)
        {
        case START:
            this->InerpolationLimbJoint(LArmJointTargetPosUp, LArm);
            this->InerpolationLimbJoint(RArmJointTargetPosUp, RArm);
            this->InerpolationLimbJoint(LLegJointTargetPosUp, LLeg);
            this->InerpolationLimbJoint(RLegJointTargetPosUp, RLeg);
            this->Time += Ts;
            if(this->Time >= this->SumMoveTimes)
            {
                this->DunStateFlag = DOWN;
                this->DunCount++;
                this->SumMoveTimes = (this->DunCount + 1) * this->MoveTimeS;
            }
            break;

        case UP:
            this->InerpolationLimbJoint(LArmJointTargetPosUp, LArm);
            this->InerpolationLimbJoint(RArmJointTargetPosUp, RArm);
            this->InerpolationLimbJoint(LLegJointTargetPosUp, LLeg);
            this->InerpolationLimbJoint(RLegJointTargetPosUp, RLeg);
            this->Time += Ts;
            if(this->Time >= this->SumMoveTimes)
            {
                this->DunStateFlag = DOWN;
                this->DunCount++;
                this->SumMoveTimes = (this->DunCount + 1) * this->MoveTimeS;
                // switch to END directly if Count achieve MAX
                if(DunCount >= DunCountMax)
                    this->DunStateFlag = END;
            }

            break;  

        case DOWN:
            this->InerpolationLimbJoint(LArmJointTargetPosDown, LArm);
            this->InerpolationLimbJoint(RArmJointTargetPosDown, RArm);
            this->InerpolationLimbJoint(LLegJointTargetPosDown, LLeg);
            this->InerpolationLimbJoint(RLegJointTargetPosDown, RLeg);
            this->Time += Ts;
            if(this->Time >= this->SumMoveTimes)
            {
                this->DunStateFlag = UP;
                this->DunCount++;
                this->SumMoveTimes = (this->DunCount + 1) * this->MoveTimeS;
                // switch to END directly if Count achieve MAX
                if(DunCount >= DunCountMax)
                    this->DunStateFlag = END;
            }
            break;
        case END:
            // return to the ini pos
            this->InerpolationLimbJoint(LArmJointIni, LArm);
            this->InerpolationLimbJoint(RArmJointIni, RArm);
            this->InerpolationLimbJoint(LLegJointIni, LLeg);
            this->InerpolationLimbJoint(RLegJointIni, RLeg);
            this->Time += Ts;
            if(this->Time >= this->SumMoveTimes)  
            {
                this->IsRunning = false;
                this->DunStateFlag = NONE;

                // Reset
                this->Time = 0.0;
                this->SumMoveTimes = this->MoveTimeS;
                this->DunCount = 0;
                std::cout<<"Reset Interp!"<<std::endl;
                return 1;
            }

        default:
            break;
        }
        
    }
    // 非插值过程，按 3 触发插值计算，并以实际关节角度为初始状态
    else if(*In.pPressKey == '3')
    {
        this->IsRunning = true;
        this->DunStateFlag = START;
        for(int i=0;i<6;i++)
        {
            this->LArmJoint.Pos(i) = this->DataInput.LArmJointRealPos[i];
            this->RArmJoint.Pos(i) = this->DataInput.RArmJointRealPos[i];
            this->LLegJoint.Pos(i) = this->DataInput.LLegJointRealPos[i];
            this->RLegJoint.Pos(i) = this->DataInput.RLegJointRealPos[i];

            this->LArmJointIni(i) = this->DataInput.LArmJointRealPos[i];
            this->RArmJointIni(i) = this->DataInput.RArmJointRealPos[i];
            this->LLegJointIni(i) = this->DataInput.LLegJointRealPos[i];
            this->RLegJointIni(i) = this->DataInput.RLegJointRealPos[i];
        }
    }
    return 0;
}

std::string Block::tostring(enum DunState DunFlag)
{
    std::string printmsg;
    switch (DunFlag)
    {
    case START:
        printmsg = "START";
        break;
    case UP:
        printmsg = "UP";   
        break; 
    case DOWN:
        printmsg = "DOWN"; 
        break;
    case END:
        printmsg = "END"; 
        break;
    }

    return printmsg;
}
int Block::print()
{
    switch (this->GUIFlag)
    {
    case GUIStateFlag::GUI_OFF :
        std::cout<<"DunStartFlag: "<< this->IsRunning <<" DunState: "<<this->tostring(this->DunStateFlag) <<std::endl;
        break;
    case GUIStateFlag::GUI_ON :
        // TODO 有待补充
        break;
    default:
        break;
    }
    
    return 0;
}

int Block::log()
{
    return 0;
}
_INTERP_TEST_END