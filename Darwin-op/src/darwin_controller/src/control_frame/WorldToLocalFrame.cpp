#include <control_frame/WorldToLocalFrame.h>
#include <iostream>
#include <Eigen/Dense>
#include <glog/logging.h>
namespace waw{namespace world2local{

Eigen::Matrix3d L_R_World(Eigen::Matrix3d::Identity());
Eigen::Vector3d World_P_L(Eigen::Vector3d::Zero());

auto getRot(const double &RX, const double &RY, const double &RZ)
{
    using namespace Eigen;
    return (AngleAxisd(RZ, Vector3d::UnitZ())*AngleAxisd(RY, Vector3d::UnitY())*AngleAxisd(RX, Vector3d::UnitX())).matrix();
}

void getAng(const Eigen::Matrix3d & _Rot, double * _Ang)
{
    _Ang[2] = atan2(_Rot(1,0), _Rot(0,0));
    _Ang[0] = atan2(_Rot(2,1), _Rot(2,2));
    _Ang[1] = -asin(_Rot(2,0));
}

void transPosFromW2L(const double *pW_Pos, double *pL_Pos)
{
    // Position
    Eigen::Vector3d W_Pos(pW_Pos);
    Eigen::Vector3d L_P = L_R_World*(W_Pos - World_P_L);
    for(int i=0;i<3;i++) pL_Pos[i] = L_P(i);
}

void transAngFromW2L(const double *pW_Ang, double *pL_Ang)
{
    // Posture
    Eigen::Matrix3d L_Rot = L_R_World*getRot(pW_Ang[0],pW_Ang[1],pW_Ang[2]);
    getAng(L_Rot, pL_Ang);
}

void transVecFromW2L(const double *pW_Vec, double *pL_Vec)
{
    Eigen::Vector3d L_Vec = L_R_World*Eigen::Vector3d(pW_Vec);
    for(int i=0;i<3;i++) pL_Vec[i] = L_Vec(i);
}

Block::Block()
{
    std::cout<<"Create Block: World To Local Frame"<<std::endl;
}

int Block::init()
{
    return 0;
}

int Block::run()
{
    auto &In = this->DataInput;
    auto &Out = this->DataOutput;

    L_R_World = Eigen::AngleAxisd(In.W_BodyAng[2],Eigen::Vector3d::UnitZ()).matrix().transpose();
    World_P_L = Eigen::Vector3d(In.W_BodyPos);

    transPosFromW2L(In.W_BodyPos    ,   Out.L_BodyPos );
    transAngFromW2L(In.W_BodyAng    ,   Out.L_BodyAng );
    
    transPosFromW2L(In.W_FootPosL   ,   Out.L_FootPosL);
    transAngFromW2L(In.W_FootAngL   ,   Out.L_FootAngL);

    transPosFromW2L(In.W_FootPosR   ,   Out.L_FootPosR);
    transAngFromW2L(In.W_FootAngR   ,   Out.L_FootAngR);

    transPosFromW2L(In.W_ZMP        ,   Out.L_ZMP     );

    transVecFromW2L(In.W_BodyLinearVel, Out.L_BodyLinearVel);
    transVecFromW2L(In.W_BodyLinearAcc, Out.L_BodyLinearAcc);

    for(int i=0;i<3;i++) Out.L_BodyAngularVel[i] = In.W_BodyAngularVel[i];  

    // LOG(INFO)<<"world to local frame:";
    // std::cout<<"input:"<<std::endl;
    // std::cout<<"W_BodyPos: "<<*In.W_BodyPos<<std::endl;
    // std::cout<<"W_BodyAng: "<<*In.W_BodyAng<<std::endl;
    // std::cout<<"W_BodyLinearVel: "<<*In.W_BodyLinearVel<<std::endl;
    // std::cout<<"W_BodyAngularVel: "<<*In.W_BodyAngularVel<<std::endl;
    // std::cout<<"W_BodyLinearAcc: "<<*In.W_BodyLinearAcc<<std::endl;
    // std::cout<<"W_FootPosL: "<<*In.W_FootPosL<<std::endl;
    // std::cout<<"W_FootAngL: "<<*In.W_FootAngL<<std::endl;
    // std::cout<<"W_FootPosR: "<<*In.W_FootPosR<<std::endl;
    // std::cout<<"W_FootAngR: "<<*In.W_FootAngR<<std::endl;
    // std::cout<<"W_ZMP: "<<*In.W_ZMP<<std::endl;
    // std::cout<<"output: "<<std::endl;
    // std::cout<<"L_BodyPos: "<<Out.L_BodyPos[0]<<" "<<Out.L_BodyPos[1]<<" "<<Out.L_BodyPos[2]<<std::endl;
    // std::cout<<"L_BodyAng: "<<Out.L_BodyAng[0]<<" "<<Out.L_BodyAng[1]<<" "<<Out.L_BodyAng[2]<<std::endl;
    // std::cout<<"L_BodyLinearVel: "<<Out.L_BodyLinearVel[0]<<" "<<Out.L_BodyLinearVel[1]<<" "<<Out.L_BodyLinearVel[2]<<std::endl;
    // std::cout<<"L_BodyAngularVel: "<<Out.L_BodyAngularVel[0]<<" "<<Out.L_BodyAngularVel[1]<<" "<<Out.L_BodyAngularVel[2]<<std::endl;
    // std::cout<<"L_BodyLinearAcc: "<<Out.L_BodyLinearAcc[0]<<" "<<Out.L_BodyLinearAcc[1]<<" "<<Out.L_BodyLinearAcc[3]<<std::endl;
    // std::cout<<"L_FootPosL: "<<Out.L_FootPosL[0]<<" "<<Out.L_FootPosL[1]<<" "<<Out.L_FootPosL[2]<<std::endl;
    // std::cout<<"L_FootAngL: "<<Out.L_FootAngL[0]<<" "<<Out.L_FootAngL[1]<<" "<<Out.L_FootAngL[2]<<std::endl;
    // std::cout<<"L_FootPosR: "<<Out.L_FootPosR[0]<<" "<<Out.L_FootPosR[1]<<" "<<Out.L_FootPosR[2]<<std::endl;
    // std::cout<<"L_FootAngR: "<<Out.L_FootAngR[0]<<" "<<Out.L_FootAngR[1]<<" "<<Out.L_FootAngR[2]<<std::endl;
    // std::cout<<"L_ZMP: "<<Out.L_ZMP[0]<<" "<<Out.L_ZMP[1]<<" "<<Out.L_ZMP[2]<<std::endl;
    return 0;
}

}}