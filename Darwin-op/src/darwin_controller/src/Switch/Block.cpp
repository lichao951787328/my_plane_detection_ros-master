#include <Switch/Block.h>
// #include <NetPath/Block.h>
#include <Remote/Block.h>
#include <iostream>
#include <glog/logging.h>
_SWITCH_BEGIN

Block::Block()
{
    std::cout<<"Create Block: Switch between Remote and Perception"<<std::endl;

    this->addBlock<lee::path::remote::Block>();
    // this->addBlock<lee::path::net_path::Block>();

}

int Block::init()
{   
    auto pRemote = this->getSubBlock<lee::path::remote::Block>(0);
    // auto pPerception = this->getSubBlock<lee::path::net_path::Block>(1);

    auto pIn = &this->DataInput;
    pRemote->setInput({
        pIn->PressKey,
        pIn->SupFlag,
        pIn->RefFootPosL,
        pIn->RefFootAngL,
        pIn->RefFootPosR,
        pIn->RefFootAngR
    });
    // pPerception->setInput({
    //     pIn->PressKey,
    //     pIn->BodyPos,
    //     pIn->BodyAng,
    //     pIn->RefFootPosL,
    //     pIn->RefFootAngL,
    //     pIn->RefFootPosR,
    //     pIn->RefFootAngR,
    //     pIn->SupFlag
    // });
    this->DataOutput.PathState = REMOTE;
    *pIn->ppFootholdList = pRemote->getOutput().pFootholdList;
    this->DataOutput.RemoteKey = pRemote->getOutput().RemoteKey;
    this->pRemoteKey = &pRemote->getOutput().RemoteKey;
    this->pFootholdListRemote = pRemote->getOutput().pFootholdList;
    // this->pFootholdListPerception = &pPerception->getOutput().FootholdList;
    LOG(INFO)<<this->SubBlockList.size();
    for(auto i:this->SubBlockList) i->init();
    std::cout<<"Init Switch"<<std::endl;
    return 0;
}

int Block::run()
{   
    LOG(INFO)<<"sub block size: "<<this->SubBlockList.size();
    auto pIn = &this->DataInput;
    auto pOut = &this->DataOutput;
    if(*pIn->WalkState == STAND)
    {
        if(*pIn->PressKey == 'r')
        {
            *pIn->ppFootholdList = this->pFootholdListRemote;
            this->pFootholdListPerception->clear();
            this->DataOutput.PathState = REMOTE;
        }
        else if(*pIn->PressKey == 'p')
        {
            *pIn->ppFootholdList = this->pFootholdListPerception;
            this->pFootholdListRemote->clear();
            pOut->PathState = PERCEPTION;
        }
    }
    switch (pOut->PathState)
    {
    case REMOTE:
        this->SubBlockList[0]->run();
        this->DataOutput.RemoteKey = *this->pRemoteKey;
        break;
    case PERCEPTION:
        this->SubBlockList[1]->run();
        this->DataOutput.RemoteKey = *pIn->PressKey;
        break;
    default:
        break;
    }
    // LOG(INFO)<<"switch block:";
    // std::cout<<"input:"<<std::endl;
    // std::cout<<"PressKey: "<<*(this->DataInput.PressKey)<<std::endl;
    // std::cout<<"WalkState: "<<*(this->DataInput.WalkState)<<std::endl;
    // std::cout<<"SupFlag: "<<*(this->DataInput.SupFlag)<<std::endl;
    // std::cout<<"BodyPos: "<<*(this->DataInput.BodyPos)<<std::endl;
    // std::cout<<"BodyAng: "<<*(this->DataInput.BodyAng)<<std::endl;
    // std::cout<<"RefFootPosL: "<<*(this->DataInput.RefFootPosL)<<std::endl;
    // std::cout<<"RefFootAngL: "<<*(this->DataInput.RefFootAngL)<<std::endl;
    // std::cout<<"RefFootPosR: "<<*(this->DataInput.RefFootPosR)<<std::endl;
    // std::cout<<"RefFootAngR: "<<*(this->DataInput.RefFootAngR)<<std::endl;
    // std::cout<<"ppFootholdList: "<<(*(*(this->DataInput.ppFootholdList))).size()<<std::endl;
    // std::cout<<"output:"<<std::endl;
    // std::cout<<"PathState: "<<(this->DataOutput.PathState)<<std::endl;
    // std::cout<<"RemoteKey: "<<(this->DataOutput.RemoteKey)<<std::endl;
    return 0;
}

int Block::print()
{
    auto pRemote = this->getSubBlock<lee::path::remote::Block>(0);
    // auto pPerception = this->getSubBlock<lee::path::net_path::Block>(1);
    using namespace ljh::tools;
    switch (this->GUIFlag)
    {
    
    case GUIStateFlag::GUI_OFF:
        std::cout<<"[Switch] ";
        if(this->DataOutput.PathState==REMOTE)
            std::cout<<"Remote     ";
        else
            std::cout<<"Perception ";
        std::cout<<std::endl;

        pRemote->setGUIFlag(ljh::tools::GUIStateFlag::GUI_OFF);
        // pPerception->setGUIFlag(ljh::tools::GUIStateFlag::GUI_OFF);
        for(auto i:this->SubBlockList) i->print();

        break;
    case GUIStateFlag::GUI_ON:

        pRemote->setGUIFlag(ljh::tools::GUIStateFlag::GUI_ON);
        // pPerception->setGUIFlag(ljh::tools::GUIStateFlag::GUI_ON);
        for(auto i:this->SubBlockList) i->print();

        ImGui::Begin("Remote&Path-steplist");
        if(ImGui::BeginTable("Remote&Path-steplist", 7))
        {
            ImU32 row_bg_color_header = ImGui::GetColorU32(ImVec4(((float)51)/255, ((float)102)/255, ((float)0.)/255, 0.65f));
            ImU32 row_bg_color = ImGui::GetColorU32(ImVec4(((float)0)/255, ((float)102)/255, ((float)51)/255, 0.65f));
            
            ImGui::TableNextRow();
            ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color_header);
            ImGui::TableNextColumn();
            ImGui::Text("StepTime");
            ImGui::TableNextColumn();
            ImGui::Text("StepFootPos-X");
            ImGui::TableNextColumn();
            ImGui::Text("StepFootPos-Y");
            ImGui::TableNextColumn();
            ImGui::Text("StepFootPos-Z");
            ImGui::TableNextColumn();
            ImGui::Text("StepFootAng-X");
            ImGui::TableNextColumn();
            ImGui::Text("StepFootAng-Y");
            ImGui::TableNextColumn();
            ImGui::Text("StepFootAng-Z");
            if(this->DataOutput.PathState == REMOTE)
            {
                for(int i=0;i<(*this->pFootholdListRemote).size();i++)
                {
                    ImGui::TableNextRow();
                    ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color);
                    ImGui::TableNextColumn();
                    ImGui::Text("%2.3f s",(*this->pFootholdListRemote)[i].StepTime);
                    ImGui::TableNextColumn();
                    ImGui::Text("%2.3f m",(*this->pFootholdListRemote)[i].Pos[0]);
                    ImGui::TableNextColumn();
                    ImGui::Text("%2.3f m",(*this->pFootholdListRemote)[i].Pos[1]);
                    ImGui::TableNextColumn();
                    ImGui::Text("%2.3f m",(*this->pFootholdListRemote)[i].Pos[2]);
                    ImGui::TableNextColumn();
                    ImGui::Text("%2.3f m",(*this->pFootholdListRemote)[i].Ang[0]);
                    ImGui::TableNextColumn();
                    ImGui::Text("%2.3f m",(*this->pFootholdListRemote)[i].Ang[1]);
                    ImGui::TableNextColumn();
                    ImGui::Text("%2.3f m",(*this->pFootholdListRemote)[i].Ang[2]);
                }

            }
            else if(this->DataOutput.PathState == PERCEPTION)
            {
                for(int i=0;i<(*this->pFootholdListPerception).size();i++)
                {
                    ImGui::TableNextRow();
                    ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color);
                    ImGui::TableNextColumn();
                    ImGui::Text("%2.3f s",(*this->pFootholdListPerception)[i].StepTime);
                    ImGui::TableNextColumn();
                    ImGui::Text("%2.3f m",(*this->pFootholdListPerception)[i].Pos[0]);
                    ImGui::TableNextColumn();
                    ImGui::Text("%2.3f m",(*this->pFootholdListPerception)[i].Pos[1]);
                    ImGui::TableNextColumn();
                    ImGui::Text("%2.3f m",(*this->pFootholdListPerception)[i].Pos[2]);
                    ImGui::TableNextColumn();
                    ImGui::Text("%2.3f m",(*this->pFootholdListPerception)[i].Ang[0]);
                    ImGui::TableNextColumn();
                    ImGui::Text("%2.3f m",(*this->pFootholdListPerception)[i].Ang[1]);
                    ImGui::TableNextColumn();
                    ImGui::Text("%2.3f m",(*this->pFootholdListPerception)[i].Ang[2]);
                }
            }
            
            ImGui::EndTable();
        }
        ImGui::End();
        break;
    default:
        break;
    }

    

    return 0;
}

int Block::log()
{
    for(auto i:this->SubBlockList) i->log();
    return 0;
}

int Block::clear()
{
    for(auto i:this->SubBlockList) i->clear();
    return 0;
}
_SWITCH_END