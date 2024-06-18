#include <control_frame/ControlFrameWalkBasic.h>
#include <Switch/Block.h>
#include <DCM_Walk/Block.h>
#include <LOrdinaryIK/IKBlock.h>
#include <BHRRobotKinematicParam/BHRRobotParameters.h>
#include <iostream>
#include <glog/logging.h>
//TODO 将IK模块升级为基于QP的全身运动学解算，输出包括手臂关节轨迹
//TODO 控制模块需要外包装（类Movement，状态机切换行走控制器和操作控制器/或者就直接wbc控制器）
//TODO 测试3D-ZMP行走轨迹和DCM的优劣，如果改为3D-ZMP，需要同步修改Switch操作的落脚点序列构造
//TODO 轨迹生成器也需要外包装（状态机切换行走轨迹规划和手臂操作轨迹规划）

_BHR_CF_BEGIN
namespace _youyi_walk_basic{
    lee::DCM_WalkPlanner::Output *pOutDCM;
    
    // PUT SWITCH AND DCM outside to use them in parameters
    lee::path::_switch::Block *pBlockSwitch;
    lee::DCM_WalkPlanner::Block *pBlockDCM;

    bool startSimLogFlag =  false;
    // Parameters Settings
    using waw::ljh::BHRConfig;
    const double TarBodyHeight  = 0.248152-0.05; //0.70;
    const double SwingFootHeight = 0.02; //0.05;
    // const lee::math::kinematics::Vec3 CoM_Bias    = {-0.005, 0.005, 0.36};
    const lee::math::kinematics::Vec3 CoM_Bias    = {0.0, 0.0, 0.0};//{0.0, 0.0, 0.36};
    const lee::math::kinematics::Vec3 CoM_Inertia = {4.0, 4.0, 0.1}; // QPFD 简化刚体转动惯量

YOUYIControlFrame::YOUYIControlFrame(const YOUYIControlFrame & other):Frame(other)
{
    
}

YOUYIControlFrame & YOUYIControlFrame::operator=(const YOUYIControlFrame & other)
{
    if(this==&other) //如果对象和other是用一个对象，直接返回本身  
    {  
        return *this;  
    }
    Frame::operator=(other);
    SupFlag = other.SupFlag;
    GUIFlag = other.GUIFlag;
    return *this;
}

std::deque<lee::math::biped::FootholdType> Path;

int YOUYIControlFrame::init()
{
    using ft=lee::math::biped::FootholdType;
    using namespace lee::math::biped;
    Path.push_back(ft(_LEFT__, 0.8, {0.00, 0.037, 0}));
    Path.push_back(ft(_RIGHT_, 0.8, {0.05,-0.037, 0}));
    Path.push_back(ft(_LEFT__, 0.8, {0.10, 0.037, 0}));
    Path.push_back(ft(_RIGHT_, 0.8, {0.15,-0.037, 0}));
    Path.push_back(ft(_LEFT__, 0.8, {0.20, 0.037, 0}));
    Path.push_back(ft(_RIGHT_, 0.8, {0.20,-0.037, 0}));


    // Construct the SubBlocks
    pBlockSwitch       = new lee::path::_switch::Block;
    pBlockDCM          = new lee::DCM_WalkPlanner::Block;
    // Block order 1
                         this->addBlock(pBlockSwitch);
    // Block order 2                     
                         this->addBlock(pBlockDCM);

    auto pBlockIK      = this->addBlock<lee::thesis::IK::Block>();
    
    //auto pBlockNumIK   = this->addBlock<chz::NumIK::NumIKBlock>();

    // Construct the Input and Output
    std::cout<<"input date: "<<std::endl;
    auto pInThis  = &this->DataInput;
         pOutDCM  = &pBlockDCM->getOutput();

    // Only for Walking
    pBlockIK->setOutput({
        this->getOutput().RefLegJointL,
        this->getOutput().RefLegJointR
    });
    
    pBlockSwitch->setInput({
        this->DataInput.PressKey  ,
        pOutDCM->WalkState        ,
        &pOutDCM->SupFlag         ,
        pOutDCM->RefBodyPos       ,
        pOutDCM->RefBodyAng       ,
        pOutDCM->RefFootPosL      ,
        pOutDCM->RefFootAngL      ,
        pOutDCM->RefFootPosR      ,
        pOutDCM->RefFootAngR      ,
        &pBlockDCM->getInput().pFootholdList});
    
    pBlockDCM->getInput().PressKey = this->DataInput.PressKey;
    pBlockDCM->getInput().pFootholdList = &Path;
    

    pBlockIK->getInput().pBodyPosition  = pOutDCM->RefBodyPos;
    pBlockIK->getInput().pBodyPosture   = pOutDCM->RefBodyAng;
    pBlockIK->getInput().pLFootPosition = pOutDCM->RefFootPosL;
    pBlockIK->getInput().pRFootPosition = pOutDCM->RefFootPosR;
    pBlockIK->getInput().pLFootPosture  = pOutDCM->RefFootAngL;
    pBlockIK->getInput().pRFootPosture  = pOutDCM->RefFootAngR;

    this->GUIFlag = ljh::tools::GUIStateFlag::GUI_OFF;

    this->setParameters();
    this->pLogger = this->getInput().pLogger;

    // DCM Settings
    pBlockDCM->setTs(this->Ts);
    pBlockDCM->setLogger(this->pLogger);
    pBlockDCM->setTarBodyHeight(TarBodyHeight);
    // pBlockDCM->setInitBodyPosture(lee::math::kinematics::Vec3{0,0,waw::ljh::BHRResetPos.getBodyIniHeight()});
    pBlockDCM->setCoM_Bias(CoM_Bias);
    pBlockDCM->setFootSwingHeight(SwingFootHeight);

    this->SupFlag = &pOutDCM->SupFlag;
    this->Frame::init();

    std::cout<<" Version Walk Basic V0: Remote and Perception"<<std::endl;
    // system("pause");
    // system("cls");
    std::cout<<"System Cycle Time:"<<this->Ts<<std::endl;
    return 0;
}

int YOUYIControlFrame::run()
{
    this->Frame::run();
    this->calRefForce(*this->SupFlag);
    for(auto i:this->SubBlockList) i->run();

    // LOG(INFO)<<"youyi control frame:";
    // std::cout<<"input:"<<std::endl;
    // std::cout<<"left leg: "<<std::endl;
    // for (size_t i = 0; i < 6; i++)
    // {
    //     std::cout<<this->DataInput.RealLegJointL[i]<<" ";
    // }
    // std::cout<<std::endl;
    // std::cout<<"right leg: "<<std::endl;
    // for (size_t i = 0; i < 6; i++)
    // {
    //     std::cout<<this->DataInput.RealLegJointR[i]<<" ";
    // }
    // std::cout<<std::endl;
    // std::cout<<"left foot force: "<<std::endl;
    // for (size_t i = 0; i < 3; i++)
    // {
    //     std::cout<<this->DataInput.FSFootForceL[i]<<" ";
    // }
    // std::cout<<std::endl;
    // std::cout<<"right foot force: "<<std::endl;
    // for (size_t i = 0; i < 3; i++)
    // {
    //     std::cout<<this->DataInput.FSFootForceR[i]<<" ";
    // }
    // std::cout<<std::endl;
    // std::cout<<"left foot torque: "<<std::endl;
    // for (size_t i = 0; i < 3; i++)
    // {
    //     std::cout<<this->DataInput.FSFootTorqueL[i]<<" ";
    // }
    // std::cout<<std::endl;
    // std::cout<<"right foot torque: "<<std::endl;
    // for (size_t i = 0; i < 3; i++)
    // {
    //     std::cout<<this->DataInput.FSFootTorqueR[i]<<" ";
    // }
    // std::cout<<std::endl;
    // std::cout<<"key: "<<*this->DataInput.PressKey<<std::endl;
    // for (size_t i = 0; i < 6; i++)
    // {
    //     std::cout<<this->DataOutput.RefLegJointL[i]<<" ";
    // }
    // std::cout<<std::endl;
    // for (size_t i = 0; i < 6; i++)
    // {
    //     std::cout<<this->DataOutput.RefLegJointR[i]<<" ";
    // }
    // std::cout<<std::endl;
    return 0;
}

int YOUYIControlFrame::log()
{
    // in simulation, start the logger directly to avoid problem in Exp
    #ifdef SIMULATION
        if(*this->DataInput.PressKey == 'm'||*this->DataInput.PressKey == 'M')
        {
            startSimLogFlag = true;     
            std::cout<<"Start Simu Log!"<<std::endl;
        }
 
        if(startSimLogFlag == true)
        {
            this->DataInput.pLogger->startLog(); 
            this->Frame::log();  
        }
    #else
    this->Frame::log(); 

    #endif  
    return 0;
}

int YOUYIControlFrame::print()
{
    using namespace ljh::tools;
    switch (this->GUIFlag)
    {
    case GUIStateFlag::GUI_OFF :
        this->Frame::print();
        break;
    case GUIStateFlag::GUI_ON :
        pBlockSwitch->setGUIFlag(GUIStateFlag::GUI_ON);
        pBlockDCM->setGUIFlag(GUIStateFlag::GUI_ON);
        
        pBlockSwitch->print();
        pBlockDCM->print();
        /* code */
        break;
    default:
        break;
    }
      
    return 0;
}

int YOUYIControlFrame::clear()
{
    if(!this->Frame::clear()) return 0;
    return 1;
}

}
_BHR_CF_End

