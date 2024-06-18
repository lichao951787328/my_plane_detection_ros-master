#include <control_frame/BHR_ControlFrame.h>
#include <Eigen/Dense>
// #include<LSimpleRoboticsMath/leeBHRLegIK.h>
extern "C"
{
    #include <LSimpleRoboticsMath/leeBHRLegIK.h>
}

#include <BHRRobotKinematicParam/BHRRobotParameters.h>
using waw::ljh::BHRConfig;
_BHR_CF_BEGIN

namespace internal
{
    //Modified For BHR8P1
    double ThighLen     = BHRConfig.ThighLen;// 0.375;//0.32;
    double CrusLen      = BHRConfig.CrusLen; //0.35;//0.32;
    double HipWidth     = BHRConfig.HipWidth;// 0.20;//0.16;
    double FootHeight   = BHRConfig.FootHeight;// 0.078;//0.112;
    auto calFootZMP(const double *f, const double *tau)
    {
        Eigen::Matrix4d zmp;
        zmp.setIdentity();
        double px = 0.0, py = 0.0, pz = 0.03;
        const double &fx = f[0], &fy = f[1], &fz = f[2];
        const double &tx = tau[0], &ty = tau[1];
        if (fz < 20.0)
        {
            zmp.block<3,1>(0,3)[0] = 0;
            zmp.block<3,1>(0,3)[1] = 0;
        }
        else
        {
            zmp.block<3,1>(0,3)[0] = (-ty - pz * fx + px * fz) / fz;
            zmp.block<3,1>(0,3)[1] = (tx - pz * fy + py * fz) / fz;
        }
        zmp.block<3,1>(0,3)[2] = -FootHeight;
        return zmp;
    }
    auto transT2Mat(const TStruct &T)
    {
        Eigen::Matrix4d Mat;
        for(int i=0;i<4;i++) for(int j=0;j<4;j++) Mat(i,j) = T.t[i][j];
        return Mat; 
    }
}

void Frame::calRealBodyZMP(const double *_BodyAng)
{
    static auto &In = this->DataInput;
    std::cout<<"FzL: "<<In.FSFootForceL[2]<<" FzR: "<<In.FSFootForceR[2]<<std::endl;
    static auto &FzL = In.FSFootForceL[2];
    static auto &FzR = In.FSFootForceR[2];
    // auto &Ang = this->RevisedAng;
    auto &Ang = _BodyAng;

    static Eigen::Matrix4d Body_T_Hip[2] = {
        internal::transT2Mat(get_move(0, internal::HipWidth/2.0,0)), internal::transT2Mat(get_move(0,-internal::HipWidth/2.0,0))
        };
    Eigen::Matrix4d Ankle_T_ZMP[2] = {
        internal::calFootZMP(In.FSFootForceL, In.FSFootTorqueL),
        internal::calFootZMP(In.FSFootForceR, In.FSFootTorqueR)
        };        
    Eigen::Matrix4d Hip_T_Ankle[2] = {
        internal::transT2Mat(get_foot_t((double *)In.RealLegJointL, internal::ThighLen, internal::CrusLen)),
        internal::transT2Mat(get_foot_t((double *)In.RealLegJointR, internal::ThighLen, internal::CrusLen))
        };
    Eigen::Matrix4d World_T_Body = 
            internal::transT2Mat(get_rot(AXIS_Z,Ang[2]))*
        internal::transT2Mat(get_rot(AXIS_Y,Ang[1]))*
        internal::transT2Mat(get_rot(AXIS_X,Ang[0]));
    Eigen::Matrix4d World_T_SingleZMP[2];
    for(int i=0;i<2;i++)
    {
        World_T_SingleZMP[i] = World_T_Body*Body_T_Hip[i]*Hip_T_Ankle[i]*Ankle_T_ZMP[i];
    }
    const auto &ZMP_L = World_T_SingleZMP[0].block<3,1>(0,3);
    const auto &ZMP_R = World_T_SingleZMP[1].block<3,1>(0,3);
    if(In.FSFootForceL[2]+In.FSFootForceR[2] > 20.0)
    {
        this->RealBodyZMP[0] = FzR/(FzL + FzR) * ZMP_R[0] + FzL/(FzL + FzR) * ZMP_L[0];
        this->RealBodyZMP[1] = FzR/(FzL + FzR) * ZMP_R[1] + FzL/(FzL + FzR) * ZMP_L[1];
    }
    else
    {
        this->RealBodyZMP[0] = 0.0;
        this->RealBodyZMP[1] = 0.0;
    }
    std::cout<<"this->RealBodyZMP[0]: "<<this->RealBodyZMP[0]<<std::endl;
    std::cout<<"this->RealBodyZMP[1]: "<<this->RealBodyZMP[1]<<std::endl;
}

_BHR_CF_End