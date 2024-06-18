#include <iostream>
#include <iomanip>
#include <DCM_Walk/Block.h>
#include <DCM_Walk/Core.hpp>
#include <tools/AlignCout.h>
#include <BHRRobotKinematicParam/BHRRobotParameters.h>
#include <cmath>
#include <LGui/LGui.h>
#include <glog/logging.h>
using waw::ljh::BHRConfig;

// Modified For BHR8P1
namespace lee{namespace DCM_WalkPlanner{
    double SwingFootHeight = 0.05;
    double TarBodyHeight = 0.65;
    // Modified For BHR8P1
    // TODO 将DCM的walk初始复位角度和上层复位模块勾连起来
    // dcm::Vec3 InitBodyPos{
    //     0.0,
    //     0.0, 
    //     BHRConfig.FootHeight + 
    //         BHRConfig.CrusLen * std::cos(BHRResetPos.getJoint5IniPos()) + 
    //         BHRConfig.ThighLen * std::cos(BHRResetPos.getJoint3IniPos())
    // };
    dcm::Vec3 InitBodyPos{
        (BHRConfig.CrusLen-BHRConfig.ThighLen)*sin(10.0*lee::math::kinematics::ToRad), 
        0, 
        BHRConfig.FootHeight+(BHRConfig.CrusLen+BHRConfig.ThighLen)*std::cos(10.0*lee::math::kinematics::ToRad)
    };

    dcm::Vec3 CoM_Bias{0.015, 0.0, 0.0};
    dcm::Vec3 TarCoM_Pos{0, 0, TarBodyHeight+CoM_Bias(2)};
    dcm::Vec3 InitCoM_Pos = InitBodyPos + CoM_Bias;
    dcm::DCM_Walk<> Planner(TarBodyHeight+CoM_Bias(2), 0.005);
    
    void Block::setFootSwingHeight(const double &_Height)
    {
        SwingFootHeight = _Height;
    }
    void Block::setTarBodyHeight(const double &_Height)
    {
        TarBodyHeight = _Height;
    }
    void Block::setCoM_Bias(const dcm::Vec3 &_Bias)
    {
        CoM_Bias = _Bias;
        TarCoM_Pos = {0,0,TarBodyHeight+_Bias(2)};
        InitCoM_Pos = InitBodyPos + CoM_Bias;
    }
    void Block::setInitBodyPosture(const dcm::Vec3 &_Pos, const dcm::Vec3 &_Ang)
    {
        InitBodyPos = _Pos;
    }

    Block::Block():StateFlag(STATE_IDLE),Time(0),Ts(0.005),GUIFlag(ljh::tools::GUIStateFlag::GUI_OFF)
    {
        InitBodyPos = {
            (BHRConfig.CrusLen - BHRConfig.ThighLen) * sin(10.0 * lee::math::kinematics::ToRad),
            0,
            BHRConfig.FootHeight + (BHRConfig.CrusLen + BHRConfig.ThighLen) * std::cos(10.0 * lee::math::kinematics::ToRad)};
        std::cout<<"InitBodyPos: "<<InitBodyPos.transpose()<<std::endl;

        this->setOutput({
            Planner.getBodyPos().Pos.data(),
            Planner.getBodyAng().Pos.data(),
            Planner.getFoot(dcm::_LEFT__).Linear.Pos.data(),
            Planner.getFoot(dcm::_RIGHT_).Linear.Pos.data(),
            Planner.getFoot(dcm::_LEFT__).Angular.Pos.data(),
            Planner.getFoot(dcm::_RIGHT_).Angular.Pos.data(),
            Planner.getVRP().Pos.data(),
            Planner.getCoM().Pos.data(),
            Planner.getBodyPos().Vel.data(),
            Planner.getBodyAng().Vel.data(),
            Planner.getBodyPos().Acc.data(),

            Planner.getFoot(dcm::_LEFT__).Linear.Vel.data(),
            Planner.getFoot(dcm::_LEFT__).Linear.Acc.data(),
            Planner.getFoot(dcm::_LEFT__).Angular.Vel.data(),
            Planner.getFoot(dcm::_LEFT__).Angular.Acc.data(),

            Planner.getFoot(dcm::_RIGHT_).Linear.Vel.data(),
            Planner.getFoot(dcm::_RIGHT_).Linear.Acc.data(),
            Planner.getFoot(dcm::_RIGHT_).Angular.Vel.data(),
            Planner.getFoot(dcm::_RIGHT_).Angular.Acc.data(),

            &Planner.getWalkState()
        });
        this->DataOutput.SupFlag = lee::math::biped::DS_SUP;
        this->DataOutput.RefDCM = Planner.getDCM().Pos.data();
        this->DataOutput.RefDCM_Vel = Planner.getDCM().Vel.data();
        this->DataOutput.RefDCM_Acc = Planner.getDCM().Acc.data();
        this->DataOutput.pTimeInStep = &Planner.TimeInStep;
        this->DataOutput.StepNo = &Planner.CurrentStep;
        this->pExtLogger = NULL;
        this->Squat.MoveTime = 5.0;
        std::cout<<"DCM Walking Planner"<<std::endl;
    }

    int Block::init()
    {
        std::cout<<"InitBodyPos1: "<<InitBodyPos.transpose()<<std::endl;

        Planner = dcm::DCM_Walk<>(TarCoM_Pos(2), this->Ts);
        Planner.setFootSwingHeight(SwingFootHeight);
        Planner.getCoM_Bias() = CoM_Bias;
        // Modified For BHR8P1
        Planner.initPlanner(TarCoM_Pos, {0.0,BHRConfig.HipWidth/2.0, 0}, {0.0, -BHRConfig.HipWidth/2.0, 0});
        Planner.setFootholdList(this->DataInput.pFootholdList);
        Planner.getCoM().Pos = InitCoM_Pos;

        std::cout<<"InitBodyPos2: "<<InitBodyPos.transpose()<<std::endl;

        // std::cout<<"Init dcm planner, "<<Planner.getFootholdList().size()<<std::endl;
        std::cout<<"Init dcm planner"<<std::endl;

        std::cout<<"InitBodyPos3: "<<InitBodyPos.transpose()<<std::endl;

        
        
        return 0;
    }
    int Block::run()
    {
        Planner.setFootholdList(this->DataInput.pFootholdList);
        LOG(INFO)<<"this->StateFlag: "<<this->StateFlag;
        switch (this->StateFlag)
        {
        case STATE_IDLE:
            if(*this->DataInput.PressKey == '7')
            {
                std::cout<<"press 7"<<std::endl;
                this->StateFlag = STATE_SQUAT;
                LOG(INFO)<<"this->StateFlag: "<<this->StateFlag;
                this->Squat.GoalTime = this->Time + this->Squat.MoveTime;
            }
            break;
        case STATE_SQUAT://蹲
            LOG(INFO)<<"squat: ";
            dcm::calRealTimeInterpolation<3>(Planner.getCoM(), Planner.getDCM(), this->Time, this->Squat.GoalTime, this->Ts);
            if (this->Time > this->Squat.GoalTime)
                this->StateFlag = STATE_DCM_WALK;
            LOG(INFO)<<"this->StateFlag: "<<this->StateFlag;
            break;
        case STATE_DCM_WALK:
            LOG(INFO)<<"STATE_DCM_WALK:";
            LOG(WARNING)<<"*this->DataInput.PressKey:"<<*this->DataInput.PressKey;
            if(*this->DataInput.PressKey == 'q')
                Planner.startWalk();
            // if(*this->DataInput.PressKey == 'e')
            //     Planner.stopWalk();
            Planner.plan();
            break;
        default:
            std::cout<<"DCM Walk Block: Error State!!!"<<std::endl;
            break;
        }
        Planner.calBodyPos(Planner.getCoM());
        this->Time+=this->Ts;
        this->DataOutput.SupFlag = Planner.getSupFlag();
        // std::cout<<"dcm walk planner:"<<std::endl;
        // std::cout<<"input:"<<std::endl;
        // std::cout<<"PressKey: "<<*this->DataInput.PressKey<<std::endl;
        // std::cout<<"footstep list: "<<this->DataInput.pFootholdList->size()<<std::endl;
        // std::cout<<"output: "<<std::endl;
        // std::cout<<"RefBodyPos: "<<*this->DataOutput.RefBodyPos<<std::endl;
        // std::cout<<"RefBodyAng: "<<*this->DataOutput.RefBodyAng<<std::endl;
        // std::cout<<"RefFootPosL: "<<*this->DataOutput.RefFootPosL<<std::endl;
        // std::cout<<"RefFootPosR: "<<*this->DataOutput.RefFootPosR<<std::endl;
        // std::cout<<"RefFootAngL: "<<*this->DataOutput.RefFootAngL<<std::endl;
        // std::cout<<"RefFootAngR: "<<*this->DataOutput.RefFootAngR<<std::endl;
        // std::cout<<"RefZMP: "<<*this->DataOutput.RefZMP<<std::endl;
        // std::cout<<"RefCoM: "<<*this->DataOutput.RefCoM<<std::endl;
        // std::cout<<"RefBodyLinearVel: "<<*this->DataOutput.RefBodyLinearVel<<std::endl;
        // std::cout<<"RefBodyAngularVel: "<<*this->DataOutput.RefBodyAngularVel<<std::endl;
        // std::cout<<"RefBodyLinearAcc: "<<*this->DataOutput.RefBodyLinearAcc<<std::endl;
        // std::cout<<"RefFootLinearVel_L: "<<*this->DataOutput.RefFootLinearVel_L<<std::endl;
        // std::cout<<"RefFootLinearAcc_L: "<<*this->DataOutput.RefFootLinearAcc_L<<std::endl;
        // std::cout<<"RefFootAngVel_L: "<<*this->DataOutput.RefFootAngVel_L<<std::endl;
        // std::cout<<"RefFootAngAcc_L: "<<*this->DataOutput.RefFootAngAcc_L<<std::endl;
        // std::cout<<"RefFootLinearVel_R: "<<*this->DataOutput.RefFootLinearVel_R<<std::endl;
        // std::cout<<"RefFootLinearAcc_R: "<<*this->DataOutput.RefFootLinearAcc_R<<std::endl;
        // std::cout<<"RefFootAngVel_R: "<<*this->DataOutput.RefFootAngVel_R<<std::endl;
        // std::cout<<"RefFootAngAcc_R: "<<*this->DataOutput.RefFootAngAcc_R<<std::endl;
        // std::cout<<"WalkState: "<<*this->DataOutput.WalkState<<std::endl;
        // std::cout<<"SupFlag: "<<this->DataOutput.SupFlag<<std::endl;
        // std::cout<<"RefDCM: "<<*this->DataOutput.RefDCM<<std::endl;
        // std::cout<<"RefDCM_Vel: "<<*this->DataOutput.RefDCM_Vel<<std::endl;
        // std::cout<<"RefDCM_Acc: "<<*this->DataOutput.RefDCM_Acc<<std::endl;
        // std::cout<<"pTimeInStep: "<<*this->DataOutput.pTimeInStep<<std::endl;
        // std::cout<<"StepNo: "<<*this->DataOutput.StepNo<<std::endl;

        this->print();
        return 0;
    }
    int Block::clear()
    {
        return 0;
    }

    #define ALIGN std::setiosflags(std::ios::right)<<std::setw(8)
    int Block::print()
    {
        using namespace ljh::tools;
        auto &Out = this->DataOutput;
        switch (this->GUIFlag)
        {
        case GUIStateFlag::GUI_OFF :
            std::cout<<"[ DCM Walk ] " <<std::fixed<<std::setprecision(4)<<std::endl
            << "SupFlag: " << Out.SupFlag <<", "
            << "Step Size: " << Planner.getFootholdList().size()<<", "
            << "Walk State: " << Planner.getWalkState()<<"   "<<std::endl
            << ALIGN << "CoM: " 
            << ALIGN << Planner.getCoM().Pos[0] << ", "
            << ALIGN << Planner.getCoM().Pos[1] << ", "
            << ALIGN << Planner.getCoM().Pos[2] << ", "
            << ALIGN << "Body: " 
            << ALIGN << Out.RefBodyPos[0] << ", "
            << ALIGN << Out.RefBodyPos[1] << ", "
            << ALIGN << Out.RefBodyPos[2] << ", "<< std::endl
            << ALIGN << "FootL: " 
            << ALIGN << Out.RefFootPosL[0] << ", "
            << ALIGN << Out.RefFootPosL[1] << ", "
            << ALIGN << Out.RefFootPosL[2] << ", "
            << ALIGN << "FootR: " 
            << ALIGN << Out.RefFootPosR[0] << ", "
            << ALIGN << Out.RefFootPosR[1] << ", "
            << ALIGN << Out.RefFootPosR[2] << ", " 
            << std::endl;
            std::cout<<SPLIT_LINE<<std::endl;
            break;
        case GUIStateFlag::GUI_ON :
            
            
            ImGui::Begin("DCM Walk Info");
            if (ImGui::BeginTable("DCM Walk Info",3))
            {
                ImU32 row_bg_color_header = ImGui::GetColorU32(ImVec4(((float)51)/255, ((float)102)/255, ((float)0.)/255, 0.65f));
                ImU32 row_bg_color = ImGui::GetColorU32(ImVec4(((float)0)/255, ((float)102)/255, ((float)51)/255, 0.65f));
                //ImU32 row_bg_color_1 = ImGui::GetColorU32(ImVec4(((float)0.)/255, ((float)51)/255, ((float)102)/255, 0.65f));

                ImGui::TableNextRow();
                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color_header);
                ImGui::TableNextColumn();
                ImGui::Text("SupFlag");
                ImGui::TableNextColumn();
                ImGui::Text("Step Size");
                ImGui::TableNextColumn();
                ImGui::Text("Walk State");

                ImGui::TableNextRow();
                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color);
                ImGui::TableNextColumn();
                ImGui::Text("%d", Out.SupFlag);
                ImGui::TableNextColumn();
                ImGui::Text("%d", Planner.getFootholdList().size());
                ImGui::TableNextColumn();
                ImGui::Text("%d", Planner.getWalkState());

                ImGui::TableNextRow();
                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color_header);
                ImGui::TableNextColumn();
                ImGui::Text("CoM-X");
                ImGui::TableNextColumn();
                ImGui::Text("CoM-Y");
                ImGui::TableNextColumn();
                ImGui::Text("CoM-Z");

                ImGui::TableNextRow();
                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color);
                for (size_t i = 0; i < 3; i++)
                {
                    ImGui::TableNextColumn();
                    ImGui::Text("%.3f",Planner.getCoM().Pos[i]);
                }
                
                ImGui::TableNextRow();
                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color_header);
                ImGui::TableNextColumn();
                ImGui::Text("Body-X");
                ImGui::TableNextColumn();
                ImGui::Text("Body-Y");
                ImGui::TableNextColumn();
                ImGui::Text("Body-Z");

                ImGui::TableNextRow();
                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color);
                for (size_t i = 0; i < 3; i++)
                {
                    ImGui::TableNextColumn();
                    ImGui::Text("%.3f",Out.RefBodyPos[i]);
                }

                ImGui::TableNextRow();
                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color_header);
                ImGui::TableNextColumn();
                ImGui::Text("FootL-X");
                ImGui::TableNextColumn();
                ImGui::Text("FootL-Y");
                ImGui::TableNextColumn();
                ImGui::Text("FootL-Z");

                ImGui::TableNextRow();
                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color);
                for (size_t i = 0; i < 3; i++)
                {
                    ImGui::TableNextColumn();
                    ImGui::Text("%.3f",Out.RefFootPosL[i]);
                }

                ImGui::TableNextRow();
                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color_header);
                ImGui::TableNextColumn();
                ImGui::Text("FootR-X");
                ImGui::TableNextColumn();
                ImGui::Text("FootR-Y");
                ImGui::TableNextColumn();
                ImGui::Text("FootR-Z");

                ImGui::TableNextRow();
                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color);
                for (size_t i = 0; i < 3; i++)
                {
                    ImGui::TableNextColumn();
                    ImGui::Text("%.3f",Out.RefFootPosR[i]);
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
        if(this->pExtLogger==NULL)
         return -1;
        auto pLog = this->pExtLogger;
        auto logVec=[&](const double *_Data, const char *_Name){
            pLog->addLog(_Data[0], std::string(_Name).append("_X").c_str());
            pLog->addLog(_Data[1], std::string(_Name).append("_Y").c_str());
            pLog->addLog(_Data[2], std::string(_Name).append("_Z").c_str());
        };
        auto pOutDCM = &this->DataOutput;
        // Reference Information:
        // -- Hip:
        logVec(pOutDCM->RefBodyPos,         "DCM_W_RefBodyPos"  );
        logVec(pOutDCM->RefBodyLinearVel,   "DCM_W_RefBodyVel"  );
        logVec(pOutDCM->RefBodyAng,         "DCM_W_RefBodyAng"  );
        // -- Left foot:
        logVec(pOutDCM->RefFootPosL,        "DCM_W_RefFootPosL" );
        logVec(pOutDCM->RefFootAngL,        "DCM_W_RefFootAngL" );
        // -- Right foot:
        logVec(pOutDCM->RefFootPosR,        "DCM_W_RefFootPosR" );
        logVec(pOutDCM->RefFootAngR,        "DCM_W_RefFootAngR" );
        // -- DCM:
        logVec(pOutDCM->RefDCM,             "DCM_W_RefDCM"      );
        // -- ZMP:
        logVec(pOutDCM->RefZMP,             "DCM_W_RefZMP"      );
        // -- CoM:
        logVec(pOutDCM->RefCoM,             "DCM_W_RefCoM"      );
        // -- Sup:
        pLog->addLog(pOutDCM->SupFlag,      "DCM_SupFlag"       );
        pLog->addLog(Planner.CurrentStep,   "DCM_StepNum"       );
        return 0;
    }
}}