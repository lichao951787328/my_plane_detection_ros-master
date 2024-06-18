#include <Remote/Block.h>
// #include <NetRemote/Block.h>
#include <iomanip>
#include <iostream>
#include <glog/logging.h>
extern "C" {
    #include <LSimpleRoboticsMath/leeKinematics.h>
}
_REMOTE_BEGIN

Block::Block():StateFlag(REMOTE_OFF),GUIFlag(ljh::tools::GUIStateFlag::GUI_OFF)
{
    this->setOutput({&this->FootholdList});
    this->getOutput().RemoteKey = 0;
    std::cout<<"Create Block: Remote"<<std::endl;
}

int Block::init()
{
    std::cout<<"Init remote, size is"<<this->FootholdList.size()<<", "
    <<"default step time is " << this->StepList[0].StepTime
    <<std::endl;
    std::cout<<"....."<<std::endl;
    // auto pNet = this->addBlock<net_remote::Block>();
    // pNet->setInput({this->DataInput.PressKey});
    std::cout<<this->SubBlockList.size()<<std::endl;
    // this->SubBlockList[0]->init();
    for (auto & SubBlock  : this->SubBlockList)
    {
        SubBlock->init();
    }
    
    return 0;
}

int Block::run()
{
    auto &Key = *this->DataInput.PressKey;
    auto &SupFlag = *this->DataInput.SupFlag;
    // auto pNet = this->getSubBlock<net_remote::Block>(0);
    // auto &NetKey = *pNet->getOutput().NetKey;
    for (auto & SubBlock : this->SubBlockList)
    {
        SubBlock->run();
    }
    
    // this->SubBlockList[0]->run();

    if(Key!=0) this->DataOutput.RemoteKey = Key;
    // else if(NetKey!=0) this->DataOutput.RemoteKey = NetKey;
    else this->DataOutput.RemoteKey = 0;

    if(this->StateFlag == REMOTE_OFF)
    {
        if(this->FootholdList.size()==0)
        {
            // Cal step list
            this->calStepholds(
                this->DataInput.RefFootPosL,
                this->DataInput.RefFootAngL,
                this->DataInput.RefFootPosR,
                this->DataInput.RefFootAngR
            );
            // Generate foothold list
            this->calFootholds();
        }
        if((Key == 'q') /* || (NetKey == 'q') */)
        {
            this->StateFlag = REMOTE_ON;
        }
        else
        {
            return -1;
        }
    }
    // Stop from remote walking mode
    if((Key == 'e' /* || NetKey == 'e' */) && SupFlag < lee::math::biped::DS_SUP)
    {
        this->stopWalk();
        return 1;
    }

    // Set command velocity according to net data
    // if(pNet->getOutput().RunFlag == true)
    // {
    //     this->CmdVel.Linear[0] = pNet->getOutput().LinearVel[0];
    //     this->CmdVel.Linear[1] = pNet->getOutput().LinearVel[1];
    //     this->CmdVel.Angular = pNet->getOutput().AngularVel[0];
    // }

    // Set command velocity according to key
    this->calCmdVel();
    // Cal reference velocity
    this->calRefVel();
    // Cal step list
    this->calStepholds(
        this->DataInput.RefFootPosL,
        this->DataInput.RefFootAngL,
        this->DataInput.RefFootPosR,
        this->DataInput.RefFootAngR
    );
    // Generate foothold list
    this->calFootholds();

    // NetKey = 0;

    return 0;
}

// TODO 修改Remote的GUI界面显示
int Block::print()
{   using namespace ljh::tools;
    // auto pNet = this->getSubBlock<net_remote::Block>(0);
    switch (this->GUIFlag)
    {
    case GUIStateFlag::GUI_OFF :
        // pNet->setGUIFlag(GUIStateFlag::GUI_OFF);

        this->SubBlockList[0]->print();

        std::cout<<"[Remote] "<<this->StateFlag<<std::endl 
        << "Forward Speed: " << this->RefVel.Linear[0]*3.6 << "("<<this->CmdVel.Linear[0]*3.6<<") km/h"
        << std::endl
        << "CmdVel: " << this->CmdVel.Linear[0] << ", "<<this->CmdVel.Linear[1]<<" m/s, "<<_DEG(this->CmdVel.Angular)<<"deg/s, "
        << "RefVel: " << this->RefVel.Linear[0] << ", "<<this->RefVel.Linear[1]<<" m/s, "<<_DEG(this->RefVel.Angular)<<"deg/s"
        << std::endl;

        std::cout<<"Step Time ("<<this->StepList.size()<<"): ";
        for(int i=0;i<this->StepList.size();i++)
        {
            std::cout<<std::setprecision(2)<<this->StepList[i].StepTime<<", ";
        }
        std::cout<<std::endl;
    
        std::cout<<"Foot Pos ("<<this->FootholdList.size()<<"): ";
        for(int i=0;i<this->FootholdList.size();i++)
        {
            std::cout<<std::setprecision(3)<<"("<<this->FootholdList[i].Pos(0)<<", "<<this->FootholdList[i].Pos(1)<<", "<<this->FootholdList[i].Pos(2)<<"), ";
        }
        std::cout<<std::endl;

        std::cout<<"Foot Ang ("<<this->FootholdList.size()<<"): ";
        for(int i=0;i<this->FootholdList.size();i++)
        {
            std::cout<<std::setprecision(3)<<"("<<_DEG(this->FootholdList[i].Ang(0))<<", "<<_DEG(this->FootholdList[i].Ang(1))<<", "<<_DEG(this->FootholdList[i].Ang(2))<<"), ";
        }
        std::cout<<std::endl;
        
        break;
    case GUIStateFlag::GUI_ON :
        // pNet->setGUIFlag(GUIStateFlag::GUI_ON);
        this->SubBlockList[0]->print();

        ImGui::Begin("Remote Block");
        if(ImGui::BeginTable("Remote", 3))
        {
            ImU32 row_bg_color_header = ImGui::GetColorU32(ImVec4(((float)51)/255, ((float)102)/255, ((float)0.)/255, 0.65f));
            ImU32 row_bg_color = ImGui::GetColorU32(ImVec4(((float)0)/255, ((float)102)/255, ((float)51)/255, 0.65f));
            ImGui::TableNextRow();
            ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color_header);
            ImGui::TableNextColumn();
            ImGui::Text("StateFlag");
            ImGui::TableNextColumn();
            ImGui::Text("Ford Ref Vel");
            ImGui::TableNextColumn();
            ImGui::Text("Ford Cmd Vel");
            

            ImGui::TableNextRow();
            ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color);
            ImGui::TableNextColumn();
            ImGui::Text("%d",this->StateFlag);
            ImGui::TableNextColumn();
            ImGui::Text("%2.3f km/h",this->RefVel.Linear[0]*3.6);
            ImGui::TableNextColumn();
            ImGui::Text("%2.3f km/h",this->CmdVel.Linear[0]*3.6);


            ImGui::TableNextRow();
            ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color_header);
            ImGui::TableNextColumn();
            ImGui::Text("CmdVel-X");
            ImGui::TableNextColumn();
            ImGui::Text("CmdVel-Y");
            ImGui::TableNextColumn();
            ImGui::Text("CmdVel-Z");

            ImGui::TableNextRow();
            ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color);
            ImGui::TableNextColumn();
            ImGui::Text("%2.3f m/s",this->CmdVel.Linear[0]);
            ImGui::TableNextColumn();
            ImGui::Text("%2.3f m/s",this->CmdVel.Linear[1]);
            ImGui::TableNextColumn();
            ImGui::Text("%2.3f deg/s",this->CmdVel.Angular*57.3);    

            ImGui::TableNextRow();
            ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color_header);
            ImGui::TableNextColumn();
            ImGui::Text("RefVel-X");
            ImGui::TableNextColumn();
            ImGui::Text("RefVel-Y");
            ImGui::TableNextColumn();
            ImGui::Text("RefVel-Z");

            ImGui::TableNextRow();
            ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color);
            ImGui::TableNextColumn();
            ImGui::Text("%2.3f m/s",this->RefVel.Linear[0]);
            ImGui::TableNextColumn();
            ImGui::Text("%2.3f m/s",this->RefVel.Linear[1]);
            ImGui::TableNextColumn();
            ImGui::Text("%2.3f deg/s",this->RefVel.Angular*57.3);  
            
            

            ImGui::EndTable();
        }

        ImGui::End();

        // ImGui::Begin("Remote-steplist");
        // if(ImGui::BeginTable("Remote-steplist", 7))
        // {
        //     ImU32 row_bg_color_header = ImGui::GetColorU32(ImVec4(((float)51)/255, ((float)102)/255, ((float)0.)/255, 0.65f));
        //     ImU32 row_bg_color = ImGui::GetColorU32(ImVec4(((float)0)/255, ((float)102)/255, ((float)51)/255, 0.65f));
            
        //     ImGui::TableNextRow();
        //     ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color_header);
        //     ImGui::TableNextColumn();
        //     ImGui::Text("StepTime");
        //     ImGui::TableNextColumn();
        //     ImGui::Text("StepFootPos-X");
        //     ImGui::TableNextColumn();
        //     ImGui::Text("StepFootPos-Y");
        //     ImGui::TableNextColumn();
        //     ImGui::Text("StepFootPos-Z");
        //     ImGui::TableNextColumn();
        //     ImGui::Text("StepFootAng-X");
        //     ImGui::TableNextColumn();
        //     ImGui::Text("StepFootAng-Y");
        //     ImGui::TableNextColumn();
        //     ImGui::Text("StepFootAng-Z");
            
        //     for(int i=0;i<this->FootholdList.size();i++)
        //     {
        //         ImGui::TableNextRow();
        //         ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color);
        //         ImGui::TableNextColumn();
        //         ImGui::Text("%2.3f s",this->FootholdList[i].StepTime);
        //         ImGui::TableNextColumn();
        //         ImGui::Text("%2.3f m",this->FootholdList[i].Pos[0]);
        //         ImGui::TableNextColumn();
        //         ImGui::Text("%2.3f m",this->FootholdList[i].Pos[1]);
        //         ImGui::TableNextColumn();
        //         ImGui::Text("%2.3f m",this->FootholdList[i].Pos[2]);
        //         ImGui::TableNextColumn();
        //         ImGui::Text("%2.3f m",this->FootholdList[i].Ang[0]);
        //         ImGui::TableNextColumn();
        //         ImGui::Text("%2.3f m",this->FootholdList[i].Ang[1]);
        //         ImGui::TableNextColumn();
        //         ImGui::Text("%2.3f m",this->FootholdList[i].Ang[2]);
        //     }

        //     ImGui::EndTable();
        // }
        // ImGui::End();
        break;
    default:
        break;
    }
    
    
    return 0;
}

void Block::stopWalk()
{
    this->CmdVel = {0,0,0};
    this->RefVel = {0,0,0};
    this->FootholdList.resize(2);
    this->LastFoothold = this->FootholdList[1];
    this->StateFlag = REMOTE_OFF;
}

int Block::clear()
{
    LOG(INFO)<<"REMOTE";
    // this->SubBlockList[0]->clear();
    LOG(INFO)<<"REMOTE DOWN";
    return 0;
}

_REMOTE_END