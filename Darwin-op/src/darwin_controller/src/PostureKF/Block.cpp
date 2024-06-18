#include <PostureKF/Block.h>
#include <PostureKF/LKalmanFilter.hpp>
#include <iostream>
#include <cmath>
#include <glog/logging.h>
namespace lee{namespace posture_kf{
    
    lee::LKalmanFilter<2, 1, 1> AngEstimator[3];

    Block::Block(const double& _Ts)
    {
        this->Ts = _Ts;
        // double Ts = 0.001;

        // 定义噪声与初始状态 
        Eigen::Matrix<double,2,1> pn{0.001,0.1};
        Eigen::Matrix<double,1,1> mn{0.01};

        // 定义系统离散状态方程 
        Eigen::Matrix<double,2,2> A;  A<<1, Ts, 0, 1;
        Eigen::Matrix<double,2,1> B{Ts*Ts*0.5, Ts};
        Eigen::Matrix<double,1,2> C{1, 0};

        // 初始化kalman滤波器 
        AngEstimator[0].init({0,0}, pn, mn, A, B, C);
        AngEstimator[1].init({0,0}, pn, mn, A, B, C);
        AngEstimator[2].init({0,0}, pn, mn, A, B, C);

        this->setOutput({this->EstimatedAng});
        for(int i=0;i<3;i++)
        {
            this->EstimatedAng[i] = 0.0;
        }

        this->GUIFlag = ljh::tools::GUIStateFlag::GUI_OFF;
    };

    int Block::init()
    {
        // 如果离散周期在初始化之前被修改，需要重新初始化滤波器
        if(std::abs(this->Ts - 0.001)>1e-5)
        {
        // 定义噪声与初始状态 
        Eigen::Matrix<double,2,1> pn{0.001,0.1};
        Eigen::Matrix<double,1,1> mn{0.01};

        // 定义系统离散状态方程 
        Eigen::Matrix<double,2,2> A;  A<<1, Ts, 0, 1;
        Eigen::Matrix<double,2,1> B{Ts*Ts*0.5, Ts};
        Eigen::Matrix<double,1,2> C{1, 0};

        // 初始化kalman滤波器 
        AngEstimator[0].init({0,0}, pn, mn, A, B, C);
        AngEstimator[1].init({0,0}, pn, mn, A, B, C);
        AngEstimator[2].init({0,0}, pn, mn, A, B, C);
        }
        return 0;
    };

    int Block::run()
    {
        for(int i=0;i<2;i++)
        {
            AngEstimator[i].predict(Eigen::Matrix<double,1,1>{0.0});
            AngEstimator[i].update(Eigen::Matrix<double,1,1>{this->DataInput.IMU_Ang[i]});
            this->EstimatedAng[i] = AngEstimator[i].getX()(0);
        }
        // LOG(INFO)<<"posture kf:";
        // std::cout<<"input:"<<std::endl;
        // for (size_t i = 0; i < 3; i++)
        // {
        //     std::cout<<this->getInput().IMU_Ang[i]<<" ";
        // }
        // std::cout<<std::endl;
        // std::cout<<"output:"<<std::endl;
        // for (size_t i = 0; i < 3; i++)
        // {
        //     std::cout<<this->getOutput().EstimatedAng[i]<<" ";
        // }
        // std::cout<<std::endl;
        return 0;
    };

    int Block::print()
    {
        static double toDeg = 180.0/3.1415926;
        using namespace ljh::tools;
        switch (this->GUIFlag)
        {
        case GUIStateFlag::GUI_OFF :           
            std::cout<<std::fixed;
            std::cout<<"[Posture Kalman Filter] "
                << this->EstimatedAng[0]*toDeg <<", " 
                << this->EstimatedAng[1]*toDeg
                << std::endl;
            break;
        case GUIStateFlag::GUI_ON :
            ImGui::Begin("Posture Kalman Filter");
            if(ImGui::BeginTable("Posture Kalman Filter",2))
            {
                ImU32 row_bg_color_header = ImGui::GetColorU32(ImVec4(((float)51)/255, ((float)102)/255, ((float)0.)/255, 0.65f));
                ImU32 row_bg_color = ImGui::GetColorU32(ImVec4(((float)0)/255, ((float)102)/255, ((float)51)/255, 0.65f));
                
                ImGui::TableNextRow();
                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color_header);
                ImGui::TableNextColumn();
                ImGui::Text("EstimatedAng[0]");
                ImGui::TableNextColumn();
                ImGui::Text("EstimatedAng[1]");

                ImGui::TableNextRow();
                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color);
                ImGui::TableNextColumn();
                ImGui::Text("%3.3f",this->EstimatedAng[0]*toDeg);
                ImGui::TableNextColumn();
                ImGui::Text("%3.3f",this->EstimatedAng[1]*toDeg);

                ImGui::TableNextRow();
                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color_header);
                ImGui::TableNextColumn();
                ImGui::Text("RevisedAng[0]");
                ImGui::TableNextColumn();
                ImGui::Text("RevisedAng[1]");
                ImGui::TableNextRow();
                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0 + 1, row_bg_color);
                ImGui::TableNextColumn();
                ImGui::Text("%3.3f",this->DataInput.IMU_Ang[0]*toDeg);
                ImGui::TableNextColumn();
                ImGui::Text("%3.3f",this->DataInput.IMU_Ang[1]*toDeg);

                ImGui::EndTable();
            
            }
            ImGui::End();
            break;
           
        default:
            break;
        }

        
        return 0;
    };
    
    int Block::log()
    {
        return 0;
    };
    
    int Block::clear()
    {
        return 0;
    };
}}
