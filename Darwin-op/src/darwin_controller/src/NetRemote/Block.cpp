#include <NetRemote/Block.h>
#include <NetRemote/DataType.h>
#include <LTCPIP/LTCPIP.hpp>
#include <thread>
#include <mutex>


_NET_REMOTE_BEGIN

namespace internal{
    constexpr int Port = 20221;
    lee::tcpip::ServerTCP<char, VelocityNetDataType> NetRecver;
    VelocityNetDataType VelData = {0,0,0,0};
    std::mutex Lock;
    int NetRunFlag;
    void getNetData()
    {
        NetRecver.init(Port);
        NetRecver.waitForConnection();
        while(NetRunFlag)
        {
            VelData = NetRecver.recvData();
            if(NetRecver.getRecvFlag() == false)
            {
                VelData = {0,0,0,0};
            }
            Sleep(1);
            NetRecver.sendData('o');
        }
        std::cout<<"Net data receiving thread quit!"<<std::endl;
    }
}

Block::Block():GUIFlag(ljh::tools::GUIStateFlag::GUI_OFF)
{
    this->setOutput({
        internal::VelData.LinearVel,
        &internal::VelData.AngularVel,
        &internal::VelData.NetKey
    });
    lee::tcpip::initSocket();
    internal::NetRunFlag = 0;
    this->DataOutput.RunFlag = false;
    std::cout<<"Create Block: NetRemote" <<std::endl;
}

Block::~Block(){lee::tcpip::clearSocket();}

int Block::init()
{
    // internal::NetRecver.init(internal::Port);
    
    return 0;
}

int Block::run()
{
    if(*this->DataInput.PressKey == '2')
    {   
        if(internal::NetRunFlag != 1)
        {
            internal::NetRunFlag = 1;
            std::thread NetRecvThread(internal::getNetData);
            NetRecvThread.detach();
            this->DataOutput.RunFlag = true;
        }
        else
        {
            internal::NetRecver.close();
            internal::NetRunFlag = 0;
            this->DataOutput.RunFlag = false;
        }
    }

    return 0;
}

int Block::clear()
{
    internal::NetRunFlag = 0;
    internal::NetRecver.close();
    Sleep(1000);
    return 0;
}

int Block::print()
{   
    using namespace ljh::tools; 
    switch (this->GUIFlag)
    {
    case GUIStateFlag::GUI_OFF:
        std::cout<<"[NetRemote]"<<std::endl;
        internal::NetRecver.printMsg();
        // std::cout<<internal::NetRecver.getMsg()<<", ";
        std::cout<<std::fixed;
        std::cout<<"Velocity: "<<this->DataOutput.LinearVel[0]<<", "<<this->DataOutput.LinearVel[1]<<", "
        <<this->DataOutput.AngularVel[0]*57.3<<", "
        <<(char)internal::VelData.NetKey
        <<"        ";
        std::cout<<std::endl;
        break;
    case GUIStateFlag::GUI_ON:
        ImGui::Begin("Remote Block");
        if (ImGui::BeginTable("NetRemote", 4))
        {
            ImU32 row_bg_color_header = ImGui::GetColorU32(ImVec4(((float)51)/255, ((float)102)/255, ((float)0.)/255, 0.65f));
            ImU32 row_bg_color = ImGui::GetColorU32(ImVec4(((float)0)/255, ((float)102)/255, ((float)51)/255, 0.65f));
            //ImU32 row_bg_color_1 = ImGui::GetColorU32(ImVec4(((float)0.)/255, ((float)51)/255, ((float)102)/255, 0.65f));
            ImGui::TableNextRow();
            ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color_header);
            ImGui::TableNextColumn();
            ImGui::Text("Message");
            ImGui::TableNextColumn();
            ImGui::Text("Vel-X");
            ImGui::TableNextColumn();
            ImGui::Text("Vel-Y");
            ImGui::TableNextColumn();
            ImGui::Text("Vel-Z");

            ImGui::TableNextRow();
            ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color);
            ImGui::TableNextColumn();
            ImGui::Text(internal::NetRecver.getMsg().c_str());
            ImGui::TableNextColumn();
            ImGui::Text("%.3f",this->DataOutput.LinearVel[0]);
            ImGui::TableNextColumn();
            ImGui::Text("%.3f",this->DataOutput.LinearVel[1]);
            ImGui::TableNextColumn();
            ImGui::Text("%.3f",this->DataOutput.AngularVel[0]*57.3);
            
            ImGui::EndTable();

        }
        ImGui::End();
        break;
    default:
        break;
    }
    
    return 0;
}

_NET_REMOTE_END