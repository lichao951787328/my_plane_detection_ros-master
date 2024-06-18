#include <LOrdinaryIK/IKBlock.h>
#include <iostream>
#include <glog/logging.h>
extern "C"
{
    #include <LSimpleRoboticsMath/leeBHRLegIK.h>
}

#include <BHRRobotKinematicParam/BHRRobotParameters.h>
using waw::ljh::BHRConfig;

using namespace lee::thesis::IK;
namespace lek = lee::eigen_kinematics;

//Modified For BHR8P1
constexpr double BASE_HIP_X  = 0.0;
double BASE_HIP_Y  = 0.0;
const double &BASE_HIP_WIDTH = BHRConfig.HipWidth;// 0.10;//0.08
constexpr double BASE_HIP_Z  = 0.0;
const double &FOOT_HEIGHT    = BHRConfig.FootHeight; //0.078;//0.112;
const double &THIGH_LEN      = BHRConfig.ThighLen; //0.375;//0.32;
const double &CRUS_LEN       = BHRConfig.CrusLen;//0.35;//0.32;

Block::Block()
{
    BASE_HIP_Y = BASE_HIP_WIDTH/2.0;
    std::cout<<"Create Block: Inverse Kinematics"<<std::endl;
}

int Block::init()
{
    double _ref_joint_deg[6] = {0, 0, -10.0, 20.0, -10.0, 0};
    for (int i = 0; i < 6; i++)
    {
        this->RefJoint[i] = _deg2rad(_ref_joint_deg[i]);
    }
    this->B_T_LHip = lek::Move(BASE_HIP_X, BASE_HIP_Y, -BASE_HIP_Z);
    this->B_T_RHip = lek::Move(BASE_HIP_X, -BASE_HIP_Y, -BASE_HIP_Z);
    this->F_T_Ankle = lek::Move(0, 0, FOOT_HEIGHT);

    std::cout << "Init IK Block: "
              << "BASE_HIP: " << BASE_HIP_X << ", "<<BASE_HIP_Y << ", "<<BASE_HIP_Z << ", "
              << "FOOT_HEIGHT: " << FOOT_HEIGHT << ", "
              << "THIGH_LEN: " << THIGH_LEN << ", "
              << "CRUS_LEN: " << CRUS_LEN << ", "
              << std::endl;
    return 0;
}

int Block::run()
{
    double __hTAnkle[4][4];
    auto getT = [](Eigen::MatT &_mat, double _res[4][4])
    {
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                _res[i][j] = _mat(i, j);
            }
        }
    };
    this->W_T_Body = lek::Move(this->DataInput.pBodyPosition) * lek::RotZYX(this->DataInput.pBodyPosture);
    this->W_T_LFoot = lek::Move(this->DataInput.pLFootPosition) * lek::RotZYX(this->DataInput.pLFootPosture);
    this->W_T_RFoot = lek::Move(this->DataInput.pRFootPosition) * lek::RotZYX(this->DataInput.pRFootPosture);

    // std::cout<<"W_T_LFoot: "<<this->W_T_LFoot.block<3,1>(0,3).transpose()<<std::endl;
    // std::cout<<"W_T_RFoot: "<<this->W_T_RFoot.block<3,1>(0,3).transpose()<<std::endl;

    // Left leg ik
    this->H_T_Ankle = (this->W_T_Body * this->B_T_LHip).inverse() * this->W_T_LFoot * this->F_T_Ankle;
    getT(this->H_T_Ankle, __hTAnkle);
    ik_leg(__hTAnkle, this->DataOutput.LegJointL, this->RefJoint, THIGH_LEN, CRUS_LEN);
    
    // Right leg ik
    this->H_T_Ankle = (this->W_T_Body * this->B_T_RHip).inverse() * this->W_T_RFoot * this->F_T_Ankle;
    getT(this->H_T_Ankle, __hTAnkle);
    ik_leg(__hTAnkle, this->DataOutput.LegJointR, this->RefJoint, THIGH_LEN, CRUS_LEN);
    // test
    // LOG(INFO)<<"IK BLOCK:";
    // std::cout<<"input:"<<std::endl;
    // std::cout<<"pBodyPosition: ";
    // for (size_t i = 0; i < 3; i++)
    // {
    //     std::cout<<this->DataInput.pBodyPosition[i]<<" ";
    // }
    // std::cout<<"pBodyPosture: ";
    // for (size_t i = 0; i < 3; i++)
    // {
    //     std::cout<<this->DataInput.pBodyPosture[i]<<" ";
    // }
    // std::cout<<std::endl;
    // std::cout<<"pBodyPosition:"<<std::endl;
    // for (size_t i = 0; i < 3; i++)
    // {
    //     std::cout<<this->DataInput.pBodyPosition[i]<<" ";
    // }
    // std::cout<<std::endl;std::cout<<"pBodyPosture:"<<std::endl;
    // for (size_t i = 0; i < 3; i++)
    // {
    //     std::cout<<this->DataInput.pBodyPosture[i]<<" ";
    // }
    // std::cout<<std::endl;
    // std::cout<<"pLFootPosition: ";
    // for (size_t i = 0; i < 3; i++)
    // {
    //     std::cout<<this->DataInput.pLFootPosition[i]<<" ";
    // }
    // std::cout<<"pLFootPosture: ";
    // for (size_t i = 0; i < 3; i++)
    // {
    //     std::cout<<this->DataInput.pLFootPosture[i]<<" ";
    // }
    // std::cout<<std::endl;
    // std::cout<<"pLFootPosture:"<<std::endl;
    // for (size_t i = 0; i < 3; i++)
    // {
    //     std::cout<<this->DataInput.pLFootPosture[i]<<" ";
    // }
    // std::cout<<std::endl;
    // std::cout<<"pRFootPosition:"<<std::endl;
    // for (size_t i = 0; i < 3; i++)
    // {
    //     std::cout<<this->DataInput.pRFootPosition[i]<<" ";
    // }
    // std::cout<<std::endl;
    // std::cout<<"pRFootPosture:"<<std::endl;
    // for (size_t i = 0; i < 3; i++)
    // {
    //     std::cout<<this->DataInput.pRFootPosture[i]<<" ";
    // }
    // std::cout<<std::endl;
    // std::cout<<"output:"<<std::endl;
    // std::cout<<"LegJointL:"<<std::endl;
    // for (size_t i = 0; i < 6; i++)
    // {
    //     std::cout<<_rad2deg(this->DataOutput.LegJointL[i])<<" ";
    // }
    // std::cout<<std::endl;    
    // for (size_t i = 0; i < 6; i++)
    // {
    //     std::cout<<_rad2deg(temp[i])<<" ";
    // }
    // std::cout<<std::endl;
    // std::cout<<"LegJointR:"<<std::endl;
    // for (size_t i = 0; i < 6; i++)
    // {
    //     std::cout<<this->DataOutput.LegJointR[i]<<" ";
    // }
    // std::cout<<std::endl;
    return 0;
}

int Block::print()
{
    return 0;
}

int Block::log()
{
#ifdef _CHECK_JOINT_LIMIT
    this->Logger.startLog();
    for(int i=0;i<6;i++)
    {
        this->Logger.addLog(this->DataOutput.LegJointL[i], std::string("LegL").append(std::to_string(i+1)).c_str());
        this->Logger.addLog(this->DataOutput.LegJointR[i], std::string("LegR").append(std::to_string(i+1)).c_str());
    }
#endif
    return 0;
}

int Block::clear()
{
#ifdef _CHECK_JOINT_LIMIT
    if(this->Logger.getNameList().size()>0)
    {
        this->Logger.saveLog("Joint.dat");
    }
#endif
    return 0;
}