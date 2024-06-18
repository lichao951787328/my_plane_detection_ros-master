#pragma once
#include <LDynamics/SDFastDefs.h>
#include <Eigen/Dense>
#include "Base.h"
#include "Math.hpp"
#define _L_DYN_BEGIN    _THESIS_TOOL_BEGIN namespace dyn{
#define _L_DYN_END      _THESIS_TOOL_END }

_L_DYN_BEGIN

// using Vec3 = Eigen::Matrix<double,3,1>;
using Mat3 = Eigen::Matrix<double,3,3>;
using math::Vec3;
using math::State3;
// constexpr int DOF = 10, NOC = 4;
template<int DOF, int NOC>
class LDynamicsEigen
{
protected:
    double t, T;
    Eigen::Matrix<int,NOC,1> ContactLink;
    Vec3 ContactLocation[NOC];
    Vec3 JointLocation[DOF];
    Eigen::Matrix<double,DOF,1> LinkMass;
    double TotalMass;

    inline Vec3 trans2LinkCoMFrame(const int &_JointNo, const Vec3 &_Pos2Joint)
    {
        return _Pos2Joint + this->JointLocation[_JointNo];
    };

public:
    struct{
        Eigen::Matrix<double,DOF,1> Pos, Vel, Acc;
    } Q;
    Eigen::Matrix<double,DOF,DOF> M;
    Eigen::Matrix<double,DOF,1> H;
    struct{
        Eigen::Matrix<double,3,DOF> A, dA, LastA;
        Vec3 h;
    } AngMom;
    struct{Vec3 Lin, Ang;} Mom;
    struct{
        struct :public State3{
            // Vec3 Pos, Vel, Acc;
            Eigen::Matrix<double,3,DOF> Jacobian, dJacobian, Jacobian_Last;
            inline void calJacobianDiff(const double &_Ts){this->dJacobian = (this->Jacobian-this->Jacobian_Last)/_Ts;};
        } Lin, Ang, Omg;
        Eigen::Matrix<double,3,3> Rot;
    }Contact[NOC], COM;

    LDynamicsEigen(const double &_T=0.005)
    {
        int contact_link[NOC] = CONTACT_BODY;
        Vec3 contact_location[NOC] = CONTACT_POS;
        Vec3 joint_location[DOF] = JOINT_POS;

        for(int i=0;i<NOC;i++)
        {
            this->ContactLink(i)     = contact_link[i];
            this->ContactLocation[i] = contact_location[i];
            this->JointLocation[i]   = joint_location[i];
        }
        this->Q.Pos.setZero();
        this->Q.Vel.setZero();
        this->Q.Acc.setZero();
        
        this->t = 0;
        this->T = _T; //Default: 0.005 ms
        sdinit();
        this->updateState(0,this->Q.Pos.data(),this->Q.Vel.data());
        this->calContactPos();
        this->calContactAng();
        this->calContactJ();
        this->calCoM();
        for(int i=0;i<DOF;i++)
        {
            sdgetmass(i,&this->LinkMass(i));
            std::cout<<"Link "<<i<<" mass: "<<this->LinkMass(i)<<std::endl;
        }
        this->TotalMass = this->LinkMass.sum();
        std::cout<<"Total Mass: "<<this->TotalMass<<std::endl;
    };
    inline void setT(const double &_T){this->T = _T;};
    inline int getContactLink(const int &_ContactID){return this->ContactLink[_ContactID];};
    inline double getTotalMass(){return this->TotalMass;};

    // Dynamics calculation function 
    void updateState(const double &_time, const double *_Q, const double *_dQ)
    {
        for(int i=0;i<DOF;i++) 
        {
            this->Q.Pos(i) = _Q[i];
            this->Q.Vel(i) = _dQ[i];
        }
        sdstate(_time, this->Q.Pos.data(), this->Q.Vel.data());
    };    
    void updateState(const double &_time)
    {
        sdstate(_time, this->Q.Pos.data(), this->Q.Vel.data());
    };
    void calM()
    {
        double _M[DOF][DOF];
        sdmassmat(_M);
        for(int i=0;i<DOF;i++) for(int j=0;j<DOF;j++) this->M(i,j) = _M[i][j];
    };
    inline void calH(){sdfrcmat(this->H.data());this->H=-this->H;};
    void calContactJ(const int &k)
    {
        auto &Obj = this->Contact[k];
        Vec3 Jvl, Jwl, Jvg, Jwg;
        Obj.Lin.Jacobian_Last = Obj.Lin.Jacobian;
        Obj.Ang.Jacobian_Last = Obj.Ang.Jacobian;
        Obj.Omg.Jacobian_Last = Obj.Omg.Jacobian;
        for(int i=0;i<DOF;i++)
        {
            sdrel2cart(i, this->ContactLink(k), this->ContactLocation[k].data(), Jvl.data(), Jwl.data());
            sdtrans(this->ContactLink(k), Jvl.data(), _Ground, Jvg.data());
            sdtrans(this->ContactLink(k), Jwl.data(), _Ground, Jwg.data());
            Obj.Lin.Jacobian.block<3,1>(0,i) = Jvg;
            Obj.Omg.Jacobian.block<3,1>(0,i) = Jwg;
        }
        Obj.Ang.Jacobian = this->getPostureJacobianInverse(Obj.Ang.Pos.data())*Obj.Omg.Jacobian;
        Obj.Lin.calJacobianDiff(this->T);
        Obj.Ang.calJacobianDiff(this->T);
        Obj.Omg.calJacobianDiff(this->T);
        Obj.Ang.Vel = this->getPostureJacobianInverse(Obj.Ang.Pos.data())*Obj.Omg.Vel;
    };
    void calContactJ()
    {
        for(int i=0;i<NOC;i++) this->calContactJ(i);
    };
    void calContactPos(const int &k)
    {
        sdpos(this->ContactLink[k], this->ContactLocation[k].data(), this->Contact[k].Lin.Pos.data());
        sdvel(this->ContactLink[k], this->ContactLocation[k].data(), this->Contact[k].Lin.Vel.data());
    };
    void calContactPos()
    {
        for(int i=0;i<NOC;i++) this->calContactPos(i);
    };
    void calContactAng(const int &k)
    {
        auto &Index = this->ContactLink(k);
        this->Contact[k].Rot = this->getRot(Index);
        this->Contact[k].Ang.Pos = this->transRot2Ang(this->Contact[k].Rot);
        this->Contact[k].Omg.Vel = this->getOmega(Index);
    };
    void calContactAng()
    {
        this->calContactAng(0);
        for(int i=1;i<NOC;i++)
        {
            if(this->ContactLink(i)==this->ContactLink(i-1))
            {
                this->Contact[i] = this->Contact[i-1];
                continue;
            }
            this->calContactAng(i);
        }
    };
    void calCoM()
    {
        double icm[3][3];
        sdsys(&this->TotalMass, this->COM.Lin.Pos.data(), icm);
    };
    void calCoM_LinJacobian()
    {
        Vec3 Jvl, Jwl, Jvg;
        double p[3] = {0,0,0};
        this->COM.Lin.Jacobian_Last = this->COM.Lin.Jacobian;
        this->COM.Lin.Jacobian.setZero();
        for(int LinkSeq = 0; LinkSeq < DOF; LinkSeq++)
        {
            for(int JointSeq = 0; JointSeq < DOF; JointSeq++)
            {
                sdrel2cart(JointSeq, LinkSeq, p, Jvl.data(), Jwl.data());
                sdtrans(LinkSeq, Jvl.data(), _Ground, Jvg.data());
                this->COM.Lin.Jacobian.block<3,1>(0,JointSeq) += Jvg * this->LinkMass(LinkSeq);
            }
        }
        this->COM.Lin.Jacobian = this->COM.Lin.Jacobian/this->TotalMass;
        this->COM.Lin.calJacobianDiff(this->T);
        this->COM.Lin.Vel = this->COM.Lin.Jacobian*this->Q.Vel;
    };
    void calAngMom()
    {
        double Energy;
        double LinMom[3];
        Vec3 h_;
        this->AngMom.LastA = this->AngMom.A;
        sdmom(LinMom, this->AngMom.h.data(), &Energy);
        double delta = 1e-6;
        auto dQ_ = this->Q.Vel;
        for(int i=0;i<dof;i++)
        {
            dQ_(i) = this->Q.Vel(i) + delta;
            sdstate(0, this->Q.Pos.data(), dQ_.data());
            sdmom(LinMom, h_.data(), &Energy);
            this->AngMom.A.block<3,1>(0,i) = (h_ - this->AngMom.h)/delta;
            dQ_(i) = this->Q.Vel(i);
        }
        this->updateState(0);
        this->AngMom.dA = (this->AngMom.A - this->AngMom.LastA)/this->T;
    };
    void calCM()
    {
        double ke;
        sdmom(this->Mom.Lin.data(), this->Mom.Ang.data(), &ke);
    };

    // SD/Fast tool functions
    Vec3 getPos(const int &_JointNo, const Vec3 &_Pos2Joint)
    {
        Vec3 res;
        sdpos(_JointNo, this->trans2LinkCoMFrame(_JointNo, _Pos2Joint).data(), res.data());
        return res;
    };
    Vec3 getAng(const int &_JointNo)
    {
        return this->transRot2Ang(this->getRot(_JointNo));
    };
    Mat3 getRot(const int &_JointNo)
    {
        double rot[3][3];
        Mat3 res;
        sdorient(_JointNo, rot);
        for(int i=0;i<3;i++) for(int j=0;j<3;j++) res(i,j) = rot[i][j];
        return res;
    };
    Vec3 getVel(const int &_JointNo, const Vec3 &_Pos2Joint)
    {
        Vec3 res;
        sdpos(_JointNo, this->trans2LinkCoMFrame(_JointNo, _Pos2Joint).data(), res.data());
        return res;
    };
    Vec3 getOmega(const int &_JointNo)
    {
        Vec3 res;
        sdangvel(_JointNo, res.data());
        return res;
    };
    Vec3 getAngVel(const int &_JointNo)
    {
        return this->getPostureJacobianInverse(this->getAng(_JointNo).data())*this->getOmega(_JointNo);
    };
    // Some useful kinematics functions. In future versions, these will be moved to files outside.
    static Vec3 transRot2Ang(const Mat3 &_Rot)
    {
        Vec3 res;
        // Rz*Ry*Rz
        res[2] = atan2(_Rot(1,0), _Rot(0,0));
        res[0] = atan2(_Rot(2,1), _Rot(2,2));
        res[1] = -asin(_Rot(2,0));
        return res;
    };
    static Mat3 getPostureJacobian(const double *_Ang)
    {
        Mat3 J;
        // Rz*Ry*Rz
        double  rx = _Ang[0];
        double  ry = _Ang[1];
        double  rz = _Ang[2];
        J(0,2) =  0.0      ;
        J(1,2) =  0.0      ;
        J(2,2) =  1.0      ;
        J(0,1) = -sin(rz)  ;
        J(1,1) =  cos(rz)  ;
        J(2,1) =  0.0      ;
        J(0,0) =  cos(ry)*cos(rz);
        J(1,0) =  cos(ry)*sin(rz);
        J(2,0) = -sin(ry)  ;
        return J;
    };
    static Mat3 getPostureJacobianInverse(const double *_Ang)
    {
        Mat3 J;
        // Rz*Ry*Rz
        double  x = _Ang[0];
        double  y = _Ang[1];
        double  z = _Ang[2];
        // J(0,0) = 1.0/(cos(y)*cos(z)+pow(sin(z),2.0));
        // J(0,1) = sin(z)/(pow(cos(y),2.0)*cos(z)+cos(y)*pow(sin(z),2.0));
        // J(0,2) = 0.0;
        // J(1,0) = -sin(z)/(cos(y)*cos(z)+pow(sin(z),2.0));
        // J(1,1) = cos(z)/(cos(y)*cos(z)+pow(sin(z),2.0));
        // J(1,2) = 0.0;
        // J(2,0) = sin(y)/(cos(y)*cos(z)+pow(sin(z),2.0));
        // J(2,1) = (sin(y)*sin(z))/(pow(cos(y),2.0)*cos(z)+cos(y)*pow(sin(z),2.0));
        // J(2,2) = 1.0;
        J << cos(z)/cos(y),         sin(z)/cos(y),          0,
            -sin(z),                cos(z),                 0,
            (cos(z)*sin(y))/cos(y), (sin(y)*sin(z))/cos(y), 1.0;
        return J;
    };
};

_L_DYN_END