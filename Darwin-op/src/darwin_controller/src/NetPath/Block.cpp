#include <NetPath/Block.h>
// #include <LTCPIP/LTCPIP.hpp>
#include <thread>
#include <mutex>
#include <NetPath/DataType.h>
#include <iomanip>
#include <LSimpleRoboticsMath/Biped.hh>
#include <Eigen/EigenKinematics.hh>
#include <algorithm>
#include <matplotlibcpp.h>
#include <NetPath/FootPolygon.h>

_NET_PATH_BEGIN
using math::biped::_LEFT__;
using math::biped::_RIGHT_;

namespace internal
{
    std::string IP = "127.0.0.1";
    int Port = 3333;
    lee::tcpip::ClientTCP<SendType, RecvType<>> Net;
    SendType SendData;
    RecvType<> RecvData;
    std::mutex Lock;
    bool NetRunFlag = false;
    bool DataRecvFlag = false;
    double MaxStepLen = 0.0;
    bool PlotFlag = false;

    void plotLocalFoothols(const RecvType<> &data)
    {
        namespace plt = matplotlibcpp;
        plt::figure();
        plt::xlabel("x");
        plt::ylabel("y");
        plt::grid(true);
        plt::set_aspect_equal();
        // plt::xlim(-0.2,1.5);
        // plt::ylim(-1.0,1.0);
        // plt::ion();
        std::vector<double> X(5), Y(5);
        std::map<std::string,std::string> FillKeyWords[2];
        FillKeyWords[0]["color"] = "lightblue";
        FillKeyWords[1]["color"] = "pink";
        double Vertex[2][4];
        for (int i = 0; i < data.Num; i++)
        {
            getFootVertex(data.Data[i].Pos, data.Data[i].Yaw, data.Data[i].IsLeft, Vertex[0], Vertex[1]);
            for(int j=0;j<4;j++)
            {
                X[j] = Vertex[0][j];
                Y[j] = Vertex[1][j];
            }
            X[4] = Vertex[0][0];
            Y[4] = Vertex[1][0];
            plt::fill(X,Y,FillKeyWords[!data.Data[i].IsLeft]);
            plt::plot(X,Y,"k");
            plt::plot({data.Data[i].Pos[0]},{data.Data[i].Pos[1]},"k.");
            plt::annotate(std::to_string(i+1), data.Data[i].Pos[0], data.Data[i].Pos[1]);
            // plt::pause(0.2);
        }
        plt::show();
    }

    auto getMaxStepLen(const RecvType<> &_L_Path)
    {
        std::vector<double> StepLenList;
        for (int i = 1; i < _L_Path.Num; i++)
        {
            StepLenList.push_back(abs(_L_Path.Data[i].Pos[0] - _L_Path.Data[i - 1].Pos[0]));
        }
        if (StepLenList.size() > 0)
            return *std::max_element(StepLenList.begin(), StepLenList.end());
        else
            return 0.0;
    }

    RecvType<> transPathFromLocal2World(const Input &_In, const RecvType<> &_L_Path)
    {
        RecvType<> W_Path;
        using namespace eigen_kinematics;
        using math::biped::_LEFT__;

        Eigen::Matrix4d W_T_Local, W_T_Path, L_T_Path;
        W_T_Local = Move(_In.BodyPos) * RotZ(_In.BodyAng[2]);
        W_T_Local(2, 3) = 0.0 + _In.BodyPos[2]-0.70;// change for Youyi Height 20230524;remember coordinate transfer with platforms 0615
        W_T_Path.setIdentity();
        L_T_Path.setIdentity();

        W_Path.Num = _L_Path.Num;
        for (int i = 0; i < W_Path.Num; i++)
        {
            W_Path.Data[i].IsLeft = _L_Path.Data[i].IsLeft;
            L_T_Path = Move(_L_Path.Data[i].Pos) * RotZ(_L_Path.Data[i].Yaw);
            W_T_Path = W_T_Local * L_T_Path;
            for (int j = 0; j < 3; j++)
            {
                W_Path.Data[i].Pos[j] = W_T_Path(j, 3);
            }
            W_Path.Data[i].Yaw = _L_Path.Data[i].Yaw + _In.BodyAng[2];
        }
        return W_Path;
    }

    void getNetData(const Input &_In)
    {
        DataRecvFlag = false;
        Net.getRecvFlag() = false;
        Net.init(IP.c_str(), Port);
        // Net.init();
        if (Net.connect())
        {
            Net.sendData(SendData);
            Sleep(1);
            auto data = Net.recvData();
            if (Net.getRecvFlag())
            {
                // std::cout<<"Num is "<<data.Num<<std::endl;
                RecvData = transPathFromLocal2World(_In, data);
                MaxStepLen = getMaxStepLen(data);
                DataRecvFlag = true;
                if(PlotFlag) plotLocalFoothols(data);
            }
        };
        NetRunFlag = false;
        std::cout << "Path data receiving thread quit!" << std::endl;
    };
}

namespace chao
{
    SendType Data;
    lee::tcpip::ClientTCP<SendType, RecvType<1>> Net;
    std::thread Thread;

    void function()
    {
        Net.init(internal::IP.c_str(), internal::Port);
        if (Net.connect())
        {
            Data.str[0] = 'C';
            Net.sendData(Data);
        };
        std::cout << "Chao synchronization thread quit!" << std::endl;
    }
}

void Block::setPlotFlag(const bool &_Flag)
{
    internal::PlotFlag = _Flag;
}

Block::Block()
{
    this->StepTime = 0.8;
    this->SendChar[0] = 'B';
    this->SendChar[1] = 'C';
    lee::tcpip::initSocket();
    std::cout << "Create Block: NetPath" << std::endl;
}

Block::~Block()
{
    lee::tcpip::clearSocket();
}

void Block::setIP(const char *_IP)
{
    internal::IP = _IP;
}
void Block::setPort(const int &_Port)
{
    internal::Port = _Port;
}
bool Block::getNetRunFlag()
{
    return internal::NetRunFlag;
}
bool Block::getDataRecvFlag()
{
    return internal::DataRecvFlag;
}

int Block::init()
{
    std::cout << "Init NetPath: remote IP is " << internal::IP << ", Port is " << internal::Port << std::endl;
    return 0;
}

int Block::run()
{
    if (*this->DataInput.KeyPress == '3')
    {
        internal::SendData.str[0] = this->SendChar[0];
        internal::SendData.str[1] = this->SendChar[1];
        for (int i = 0; i < 3; i++)
        {
            internal::SendData.BodyAng[i] = this->DataInput.BodyAng[i];
            internal::SendData.BodyPos[i] = 0.0;
        }
        internal::SendData.BodyAng[2] = 0.0;
        internal::SendData.BodyPos[2] = this->DataInput.BodyPos[2];

        if (!internal::NetRunFlag)
        {
            internal::NetRunFlag = true;
            //turn off the footstep plot 
            this->setPlotFlag(false);
            std::thread NetThread(internal::getNetData, std::ref(this->DataInput));
            NetThread.detach();
        }
        else
        {
            internal::Net.close();
        }
    }

    if (*this->DataInput.KeyPress == 'q')
    {
        chao::Net.close();
        chao::Thread = std::thread(chao::function);
        chao::Thread.detach();
    }

    if (internal::DataRecvFlag)
    {
        using math::biped::Vec3;
        auto &sup_flag = *this->DataInput.SupFlag;
        auto &foot_list = this->DataOutput.FootholdList;
        auto &in = this->DataInput;
        auto is_left = internal::RecvData.Data[0].IsLeft;

        // Robot is stand; there is no steps in FootholdList
        if (is_left == true)
            foot_list.push_back({_RIGHT_, this->StepTime, Vec3{in.FootPosR}, Vec3{in.FootAngR}});
        else
            foot_list.push_back({_LEFT__, this->StepTime, Vec3{in.FootPosL}, Vec3{in.FootAngL}});

        for (int i = 0; i < internal::RecvData.Num; i++)
        {
            auto &data = internal::RecvData.Data[i];
            this->DataOutput.FootholdList.push_back({1 - (int)data.IsLeft,
                                                     this->StepTime,
                                                     {data.Pos[0], data.Pos[1], data.Pos[2]},
                                                     {0, 0, data.Yaw}});
        }
        internal::DataRecvFlag = false;
    }
    return 0;
}

int Block::print()
{
    auto &Step = this->DataOutput.FootholdList;
    using namespace ljh::tools;
    switch (this->GUIFlag)
    {
    case GUIStateFlag::GUI_OFF :
        std::cout << "[NetPath]" << std::endl;
        internal::Net.printMsg();
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "Max Step Len: " << internal::MaxStepLen << std::endl;
        std::cout<< "Num step: "<<internal::RecvData.Num<<std::endl;
        //std::cout << "Net Step: ";
        //for (int i = 0; i < internal::RecvData.Num; i++)
        //{
        //    std::cout << "("
        //              << internal::RecvData.Data[i].IsLeft << ", "
        //              << internal::RecvData.Data[i].Pos[0] << ", "
        //              << internal::RecvData.Data[i].Pos[1] << ", "
        //              << internal::RecvData.Data[i].Pos[2] << ", "
        //              << internal::RecvData.Data[i].Yaw * 57.3 << "), ";
        //    if ((i + 1) % 2 == 0)
        //        std::cout << std::endl
        //                  << "      ";
        //}
        //std::cout << std::endl;

        std::cout << "Step: ";
        for (int i = 0; i < Step.size(); i++)
        {
            std::cout << "("
                      << Step[i].FootFlag << ", "
                      << Step[i].Pos(0) << ", "
                      << Step[i].Pos(1) << ", "
                      << Step[i].Pos(2) << ", "
                      << Step[i].Ang(2) * 57.3 << "), ";
            if ((i + 1) % 2 == 0)
                std::cout << std::endl
                          << "      ";
        }
        std::cout << std::endl;
            break;
            
    case GUIStateFlag::GUI_ON :
        /* code */
        break;
    default:
        break;
    }
    
    return 0;
}

int Block::clear()
{
    internal::Net.close();
    chao::Net.close();
    Sleep(50);
    return 0;
}

_NET_PATH_END