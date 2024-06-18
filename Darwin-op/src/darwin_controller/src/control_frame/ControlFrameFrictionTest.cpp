#include <control_frame/ControlFrameFrictionTest.h>
#include <BHRRobotKinematicParam/BHRRobotParameters.h>

// include the Block.h list 

#include <iostream>
#include <string>
_BHR_CF_BEGIN

namespace FrictionTest{

bool startSimLogFlag;

ControlFrame::ControlFrame(const ControlFrame & other):Frame(other)
{
    GUIFlag = other.GUIFlag;
    for (size_t i = 0; i < 6; i++)
    {
        epsDeg[i] = other.epsDeg[i];
        MinLLegJointDeg[i] = other.MinLLegJointDeg[i];
        MaxLLegJointDeg[i] = other.MaxLLegJointDeg[i];
        MinRLegJointDeg[i] = other.MinRLegJointDeg[i];
        MaxRLegJointDeg[i] = other.MaxRLegJointDeg[i];

        LLegAmptitudeSin[i] = other.LLegAmptitudeSin[i];
        RLegAmptitudeSin[i] = other.RLegAmptitudeSin[i];
        TSin[i] = other.TSin[i];
        TimeCount[i] = other.TimeCount[i];

        LLegAngleIni[i] = other.LLegAngleIni[i];
        RLegAngleIni[i] = other.RLegAngleIni[i];

        LLegAngle[i] = other.LLegAngle[i];
        RLegAngle[i] = other.RLegAngle[i];

        SinStart[i] = other.SinStart[i];
        SinTime[i] = other.SinTime[i];
    }
    SinStartAll = other.SinStartAll;
    iniFlag = other.iniFlag;
    CrtlState = other.CrtlState;
    VelState = other.VelState;
    VelSeries = other.VelSeries;
}

ControlFrame & ControlFrame::operator=(const ControlFrame & other)
{
    if(this==&other) //如果对象和other是用一个对象，直接返回本身  
    {  
        return *this;  
    }
    Frame::operator=(other);
    GUIFlag = other.GUIFlag;
    for (size_t i = 0; i < 6; i++)
    {
        epsDeg[i] = other.epsDeg[i];
        MinLLegJointDeg[i] = other.MinLLegJointDeg[i];
        MaxLLegJointDeg[i] = other.MaxLLegJointDeg[i];
        MinRLegJointDeg[i] = other.MinRLegJointDeg[i];
        MaxRLegJointDeg[i] = other.MaxRLegJointDeg[i];

        LLegAmptitudeSin[i] = other.LLegAmptitudeSin[i];
        RLegAmptitudeSin[i] = other.RLegAmptitudeSin[i];
        TSin[i] = other.TSin[i];
        TimeCount[i] = other.TimeCount[i];

        LLegAngleIni[i] = other.LLegAngleIni[i];
        RLegAngleIni[i] = other.RLegAngleIni[i];

        LLegAngle[i] = other.LLegAngle[i];
        RLegAngle[i] = other.RLegAngle[i];

        SinStart[i] = other.SinStart[i];
        SinTime[i] = other.SinTime[i];
    }
    SinStartAll = other.SinStartAll;
    iniFlag = other.iniFlag;
    CrtlState = other.CrtlState;
    VelState = other.VelState;
    VelSeries = other.VelSeries;
    return *this;
}

int ControlFrame::init()
{
    // Construct the SubBlocks
    
    auto pBlockJointInterp = this->addBlock(new ljh::ctrl::joint_interpolation::Block(this->Ts));

    // Construct the Input and Output
    auto pInThis  = &this->DataInput;
    auto pOutThis  = &this->DataOutput;
    
    // Interpolation Settings
    pBlockJointInterp->setInput({
        this->DataInput.PressKey,
        this->JointRealPos
    });

    pBlockJointInterp->setOutput({
        this->JointRefPos
    });

    this->pLogger = this->getInput().pLogger;
    // Frame init
    this->Frame::init();
    
    this->setGUIFlag(ljh::tools::GUIStateFlag::GUI_ON);
    this->CrtlState = CONTROL_STATE_SECOND_RESET;
    pBlockJointInterp->setMoveTimes(10.0);
    this->loadTargetPosForSecondReset();

    std::cout<<"LLegAngleIni: ";
    for(int i=0;i<6;i++)
        std::cout<<LLegAngleIni[i] / PI_ljh * 180.<<" ";
    std::cout<<std::endl;
    std::cout<<"RLegAngleIni: ";
    for(int i=0;i<6;i++)
        std::cout<<RLegAngleIni[i] / PI_ljh * 180.<<" ";
    std::cout<<std::endl;
    std::cout<<" Version Friction Test"<<std::endl;
    system("pause");
    std::cout<<"System Cycle Time:"<<this->Ts<<std::endl;
    return 0;
}


int ControlFrame::run()
{
    auto pInThis  = &this->DataInput;
    auto pOutThis  = &this->DataOutput;

    this->Frame::run();
    this->runControl();
    // for(auto i:this->SubBlockList) i->run();
    
    return 0;
}

int ControlFrame::log()
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

int ControlFrame::clear()
{
    if(!this->Frame::clear()) return 0;
    return 1;
}

int ControlFrame::print()
{
    auto pBlockJointInterp = this->getSubBlock<ljh::ctrl::joint_interpolation::Block>(0);
    if(this->GUIFlag == ljh::tools::GUIStateFlag::GUI_OFF)
    {
        pBlockJointInterp->setGUIFlag(ljh::tools::GUIStateFlag::GUI_OFF);
        this->Frame::print();

        std::cout<<" [RunSinFuc]: ";

        for(int i=0;i<6;i++)
        {
          if(this->SinStart[i])
            std::cout<< " Running, "<<std::endl;
          else
            std::cout<< " NotRunning, "<<std::endl;  
        }
        
        
        std::cout<<" [crtlState]: ";
        if(this->CrtlState == CONTROL_STATE_SECOND_RESET)
            std::cout<< " CONTROL_STATE_SECOND_RESET "<<std::endl;
        else if(this->CrtlState == CONTROL_STATE_RUN_SIN_FUNC)
            std::cout<< " CONTROL_STATE_RUN_SIN_FUNC "<<std::endl;
        else
            std::cout<<" CONTROL_STATE_THIRD_RESET "<<std::endl;
        
        std::cout<<" [Tcos(sec)]: ";
        for(int i=0;i<6;i++)
            std::cout<< this->TSin[i]<<", ";
        std::cout<< std::endl;
        
        
        std::cout<<" [VelState]: "<<this->SpeedtoString()<<std::endl;

        std::cout<<"RefLeftLegJoint: ";
        for(int i=0;i<6;i++)
        {
            std::cout<<this->JointRefPos[lee::tools::Left_Leg1 + i] / PI_ljh * 180.<<", ";
        }
        std::cout<< std::endl;

        std::cout<<"RefRightLegJoint: ";
        for(int i=0;i<6;i++)
        {
            std::cout<<this->JointRefPos[lee::tools::RightLeg1 + i] / PI_ljh * 180.<<", ";
        }
        std::cout<< std::endl;

         std::cout<<"RealLeftLegJoint: ";
        for(int i=0;i<6;i++)
        {
            std::cout<<this->JointRealPos[lee::tools::Left_Leg1 + i] / PI_ljh * 180.<<", ";
        }
        std::cout<< std::endl;

        std::cout<<"RealRightLegJoint: ";
        for(int i=0;i<6;i++)
        {
            std::cout<<this->JointRealPos[lee::tools::RightLeg1 + i] / PI_ljh * 180.<<", ";
        }
        std::cout<< std::endl;

        std::cout<<"SinLeftLegJoint: ";
        for(int i=0;i<6;i++)
        {
            std::cout<<this->LLegAngle[i] / PI_ljh * 180.<<", ";
        }
        std::cout<< std::endl;

        std::cout<<"SinRightLegJoint: ";
        for(int i=0;i<6;i++)
        {
            std::cout<<this->RLegAngle[i] / PI_ljh * 180.<<", ";
        }
        std::cout<< std::endl;
    
        std::cout<<"ThirdResetTargetLeftLegJoint: ";
        for(int i=0;i<6;i++)
        {
            std::cout<<this->JointRealPosIni[lee::tools::Left_Leg1+i] / PI_ljh * 180.<<", ";
        }
        std::cout<< std::endl;

        std::cout<<"ThirdResetTargetRightLegJoint: ";
        for(int i=0;i<6;i++)
        {
            std::cout<<this->JointRealPosIni[lee::tools::RightLeg1+i] / PI_ljh * 180.<<", ";
        }
        std::cout<< std::endl;
    }
    else
    {

        pBlockJointInterp->setGUIFlag(ljh::tools::GUIStateFlag::GUI_ON);
        pBlockJointInterp->print();

        std::string ControlStateMsg;
        std::string RunSinMsg[6];
        for(int i=0;i<6;i++)
        {
            if(this->SinStart[i])
                RunSinMsg[i] = "Running ";
            else    
                RunSinMsg[i] = "NotRunning ";
        }
        if(this->CrtlState == CONTROL_STATE_SECOND_RESET)
            ControlStateMsg =  " CONTROL_STATE_SECOND_RESET "; //<<std::endl;
        else if(this->CrtlState == CONTROL_STATE_RUN_SIN_FUNC)
            ControlStateMsg =  " CONTROL_STATE_RUN_SIN_FUNC "; //<<std::endl;
        else
            ControlStateMsg = " CONTROL_STATE_THIRD_RESET "; //<<std::endl;
        // 有待补充
        ImGui::Begin("Friction Test");
        if (ImGui::BeginTable("Friction-Test", 6))
        {
            ImU32 row_bg_color_header = ImGui::GetColorU32(ImVec4(((float)51)/255, ((float)102)/255, ((float)0.)/255, 0.65f));
            ImU32 row_bg_color = ImGui::GetColorU32(ImVec4(((float)0)/255, ((float)102)/255, ((float)51)/255, 0.65f));
            //ImU32 row_bg_color_1 = ImGui::GetColorU32(ImVec4(((float)0.)/255, ((float)51)/255, ((float)102)/255, 0.65f));
            ImGui::TableNextRow();
            ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color_header);
            
            ImGui::TableNextColumn();
            ImGui::Text("crtlState");
            ImGui::TableNextColumn();
            ImGui::Text("VelState");
            
            ImGui::TableNextRow();
        	ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color);
        	ImGui::TableNextColumn();
        	ImGui::Text(ControlStateMsg.c_str());
        	ImGui::TableNextColumn();
        	ImGui::Text(this->SpeedtoString().c_str());
            
            ImGui::TableNextRow();
            ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color_header);
            ImGui::TableNextColumn();
            ImGui::Text("RunSinFuc-1");
            ImGui::TableNextColumn();
            ImGui::Text("RunSinFuc-2");
            ImGui::TableNextColumn();
            ImGui::Text("RunSinFuc-3");
            ImGui::TableNextColumn();
            ImGui::Text("RunSinFuc-4");
            ImGui::TableNextColumn();
            ImGui::Text("RunSinFuc-5");
            ImGui::TableNextColumn();
            ImGui::Text("RunSinFuc-6");

            ImGui::TableNextRow();
        	ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color);
            for(int i;i<6;i++)
            {
                ImGui::TableNextColumn();
        	    ImGui::Text(RunSinMsg[i].c_str());
            }
        	
        	
            ImGui::TableNextRow();
            ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color_header);
            ImGui::TableNextColumn();
            ImGui::Text("Tcos(sec)-1");
            ImGui::TableNextColumn();
            ImGui::Text("Tcos(sec)-2");
            ImGui::TableNextColumn();
            ImGui::Text("Tcos(sec)-3");
            ImGui::TableNextColumn();
            ImGui::Text("Tcos(sec)-4");
            ImGui::TableNextColumn();
            ImGui::Text("Tcos(sec)-5");
            ImGui::TableNextColumn();
            ImGui::Text("Tcos(sec)-6");

            ImGui::TableNextRow();
        	ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color);
            for(int i=0;i<6;i++)
            {
                ImGui::TableNextColumn();
        	    ImGui::Text("%.3f", this->TSin[i]);
            }
            
            ImGui::EndTable();
            
        }
        
    }
    
    return 0;
}

int ControlFrame::runSin()
{
    if(this->SinStartAll)
    {
        this->SinStartAll = false;
        for(int i=0;i<6;i++)
        {
            if(this->SinStart[i] == true) 
            {
            
                this->LLegAngle[i] = this->LLegAngleIni[i] +
                     this->LLegAmptitudeSin[i] *  std::cos(2*PI_ljh/this->TSin[i] * this->SinTime[i]);
                this->RLegAngle[i] = this->RLegAngleIni[i] +
                     this->RLegAmptitudeSin[i] *  std::cos(2*PI_ljh/this->TSin[i] * this->SinTime[i]);

                // TODO check before value-pass
                this->JointRefPos[lee::tools::Left_Leg1 + i] = this->LLegAngle[i];
                this->JointRefPos[lee::tools::RightLeg1 + i] = this->RLegAngle[i];

                this->SinTime[i] += this->Ts; 
                if(this->SinTime[i] >= this->TimeCount[i])
                {
                    this->SinStart[i] = false;
                }
            }
            //quit when all the cos movement finished
            this->SinStartAll += this->SinStart[i];
        }  
        if(this->SinStartAll == false) return 1;      
    }
    else if(*this->DataInput.PressKey == '2')
    {
        this->SinStartAll = true;
        for(int i=0;i<6;i++)
        {
            this->SinStart[i] = true;
            this->SinTime[i] = 0.0;
        }
        
    }
    return 0;
}

void ControlFrame::updateJointRefPos()
{
    for(int i=0;i<6;i++)
    {
        this->DataOutput.RefArmJointL[i] = this->JointRefPos[lee::tools::Left_Arm1 + i];
        this->DataOutput.RefArmJointR[i] = this->JointRefPos[lee::tools::RightArm1 + i];
        this->DataOutput.RefLegJointL[i] = this->JointRefPos[lee::tools::Left_Leg1 + i];
        this->DataOutput.RefLegJointR[i] = this->JointRefPos[lee::tools::RightLeg1 + i];
    }
    
}

void ControlFrame::updateJointRealPos()
{
    for(int i=0;i<6;i++)
    {
        this->JointRealPos[lee::tools::Left_Arm1 + i] = this->DataInput.RealArmJointL[i]; 
        this->JointRealPos[lee::tools::RightArm1 + i] = this->DataInput.RealArmJointR[i];
        this->JointRealPos[lee::tools::Left_Leg1 + i] = this->DataInput.RealLegJointL[i];
        this->JointRealPos[lee::tools::RightLeg1 + i] = this->DataInput.RealLegJointR[i];
    }
}

void ControlFrame::iniJointRefPos()
{
    for(int i=0;i<6;i++)
    {
        // 用参考轨迹初始化是为了保证空跑的时候轨迹连续（真实轨迹是一直保持不动的，不能用其进行赋值）
        // 并且上电第一段复位后，参考轨迹和真实轨迹是相同的，所以采用参考轨迹初始化，并且JointInterpolation使用的初始关节角是参考轨迹
        this->JointRefPos[lee::tools::Left_Arm1 + i] =  this->DataOutput.RefArmJointL[i]; 
        this->JointRefPos[lee::tools::RightArm1 + i] =  this->DataOutput.RefArmJointR[i]; 
        this->JointRefPos[lee::tools::Left_Leg1 + i] =  this->DataOutput.RefLegJointL[i]; 
        this->JointRefPos[lee::tools::RightLeg1 + i] =  this->DataOutput.RefLegJointR[i]; 
    }
    
}

void ControlFrame::runControl()
{
    this->updateJointRealPos();
    switch (this->CrtlState)
    {
    case CONTROL_STATE_SECOND_RESET:
        if(this->SubBlockList[0]->run())
            this->CrtlState = CONTROL_STATE_RUN_SIN_FUNC;
        break;
    case CONTROL_STATE_RUN_SIN_FUNC:
        
        // Allow key-vel-modification only before the cos movement.
        if(this->SinStartAll == false)
        {
            this->modifyVelocityWithKey(*this->DataInput.PressKey);
            for(int i=0;i<6;i++)
            {   
                // modify the cycle time of the cos according to velstate
                // when there is difference/Asymmetry in LLegAmptitudeSin and RLegAmptitudeSin, the follow line should be modified!
                this->TSin[i] = 2 * PI_ljh / (((static_cast<int>(this->VelState) + 1 ) * 0.5)/this->LLegAmptitudeSin[i]);
                this->TimeCount[i] = 2 * this->TSin[i];
            }
            
        }
            
        if(this->runSin())
        {
            this->CrtlState = CONTROL_STATE_THIRD_RESET;
            this->loadTargetPosForThirdReset();
        }
    case CONTROL_STATE_THIRD_RESET:
        if(this->SubBlockList[0]->run())
        {
            this->CrtlState = CONTROL_STATE_SECOND_RESET;
            this->loadTargetPosForSecondReset();
        }

    default:
        break;
    }

    if(this->iniFlag)
    {
        this->iniJointRefPos();
        this->updateJointRealPosIni();
        this->iniFlag = false;
    }
    this->updateJointRefPos();
    // this->log();
}

void ControlFrame::updateJointRealPosIni()
{
    for(int i=0;i<ljh::ctrl::joint_interpolation::JOINT_NUM;i++)
    {
        this->JointRealPosIni[i] = this->JointRefPos[i];
    }
    
}

void ControlFrame::loadTargetPosForThirdReset()
{
    auto pBlockJointInterp = this->getSubBlock<ljh::ctrl::joint_interpolation::Block>(0);
    for(int i=0;i<6;i++)
    {
        pBlockJointInterp->setSingleJointTargetPos(lee::tools::Left_Leg1 + i, this->JointRealPosIni[lee::tools::Left_Leg1 + i]);
        pBlockJointInterp->setSingleJointTargetPos(lee::tools::RightLeg1 + i, this->JointRealPosIni[lee::tools::RightLeg1 + i]);
    }
}

void ControlFrame::loadTargetPosForSecondReset()
{
    auto pBlockJointInterp = this->getSubBlock<ljh::ctrl::joint_interpolation::Block>(0);
    for(int i=0;i<6;i++)
    {
        pBlockJointInterp->setSingleJointTargetPos(lee::tools::Left_Leg1 + i, this->LLegAngleIni[i] + this->LLegAmptitudeSin[i]);
        pBlockJointInterp->setSingleJointTargetPos(lee::tools::RightLeg1 + i, this->RLegAngleIni[i] + this->RLegAmptitudeSin[i]);
    }
}

void ControlFrame::modifyVelocityWithKey(int key)
{
    auto Iter = this->VelSeries.begin() + this->VelState;
    switch (key)
    {
    case 'A':
    case 'a':
        if(Iter!=this->VelSeries.end()-1)
            Iter++;
        else
            Iter = this->VelSeries.begin();

        this->VelState = *Iter;

        break;
    case 'D':
    case 'd':
        if(Iter!=this->VelSeries.begin())
            Iter--;
        else
            Iter = this->VelSeries.end()-1;

        this->VelState = *Iter;
        break;
    default:
        break;
    }

}

std::string ControlFrame::SpeedtoString()
{
    std::string velStateStr = "error";
    switch (this->VelState)
    {
    case VEL_ZEROFIVE:
        velStateStr = "0.5 rad/s";
        break;
    case VEL_ONEZERO:
        velStateStr = "1.0 rad/s";
        break;
    case VEL_ONEFIVE:
        velStateStr = "1.5 rad/s";
       break; 
    case VEL_TWOZERO:
        velStateStr = "2.0 rad/s";
        break;
    case VEL_TWOFIVE:
        velStateStr = "2.5 rad/s";
        break;
    case VEL_THREEZERO:
        velStateStr = "3.0 rad/s";
        break; 
    case VEL_THREEFIVE:
        velStateStr = "3.5 rad/s";
        break;
    case VEL_FOURZERO:
        velStateStr = "4.0 rad/s";
        break;
    case VEL_FOURFIVE:
        velStateStr = "4.5 rad/s";
       break; 
    case VEL_FIVEZERO:
        velStateStr = "5.0 rad/s";
        break;
    case VEL_FIVEFIVE:
        velStateStr = "5.5 rad/s";
        break;
    case VEL_SIXZERO:
        velStateStr = "6.0 rad/s";
        break;   
    case VEL_SIXFIVE:
        velStateStr = "6.5 rad/s";
        break;
    case VEL_SEVENZERO:
        velStateStr = "7.0 rad/s";
        break;
    case VEL_SEVENFIVE:
        velStateStr = "7.5 rad/s";
       break; 
    case VEL_EIGHTZERO:
        velStateStr = "8.0 rad/s";
        break;
    case VEL_EIGHTFIVE:
        velStateStr = "8.5 rad/s";
        break;
    case VEL_NINEZERO:
        velStateStr = "9.0 rad/s";
        break;  
    case VEL_NINEFIVE:
        velStateStr = "9.5 rad/s";
        break;
    case VEL_TENZERO:
        velStateStr = "10.0 rad/s";
        break;  
    case VEL_TENFIVE:
        velStateStr = "10.5 rad/s";
        break;
    default:
        break;
    }
    return velStateStr;
}

}
_BHR_CF_End