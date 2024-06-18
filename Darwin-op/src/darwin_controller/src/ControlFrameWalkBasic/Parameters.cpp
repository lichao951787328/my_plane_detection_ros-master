#include <control_frame/ControlFrameWalkBasic.h>
#include <Switch/Block.h>
// #include <NetPath/Block.h>
#include <Remote/Block.h>
#include <DCM_Walk/Block.h>
#include <BHRRobotKinematicParam/BHRRobotParameters.h>
// #include <Compliance/CHZ/CHZBlock.h>
// #include <iostream>
_BHR_CF_BEGIN
namespace _youyi_walk_basic{
    extern lee::path::_switch::Block *pBlockSwitch;
    extern lee::DCM_WalkPlanner::Block *pBlockDCM;
void YOUYIControlFrame::setParameters()
{
    // 设置是否开启各模块的GUI显示窗口
    this->setGUIFlag(ljh::tools::GUIStateFlag::GUI_ON);  
    // 设置DCM行走时的质心偏置、上身（髋关节）高度, 默认为[0.015,0,0.16], 0.65 
    // // pBlockDCM->setTarBodyHeight(0.70);
    // // pBlockDCM->setInitBodyPosture(lee::math::kinematics::Vec3{0,0,waw::ljh::BHRResetPos.getBodyIniHeight()});
    // // pBlockDCM->setCoM_Bias({0.00, 0.005, 0.36});
    
    // // 设置DCM行走的控制周期s
    // pBlockDCM->setTs(this->Ts);

    // 设置遥控行走步行周期, 默认0.6s 
    const double walkStepTime = 0.8;
    auto pBlockRemote = pBlockSwitch->getSubBlock<lee::path::remote::Block>(0);
    pBlockRemote->setStepTime(walkStepTime);

    // 设置遥控行走路径生成控制周期
    pBlockRemote->setTs(this->Ts);

    // 先统一屏蔽感知端
    // 感知路径接收模块参数设置 
    // auto pBlockNetPath = pBlockSwitch->getSubBlock<lee::path::net_path::Block>(1);
    // // 设置感知服务器IP地址, 默认地址为 127.0.0.1，本地地址 
    // pBlockNetPath->setIP("192.168.1.100");
    // // 设置向感知服务器发送的字符，默认为 BC 
    // pBlockNetPath->setSendChar('B','C');
    // // 设置接收路径后是否图形化显示, 默认为false，不显示（工控机上显示速度很慢）  
    // pBlockNetPath->setPlotFlag(true);
    // // 设置感知路径的步行周期, 默认0.8s 
    // pBlockNetPath->setStepTime(0.8);
}

}
_BHR_RP_END