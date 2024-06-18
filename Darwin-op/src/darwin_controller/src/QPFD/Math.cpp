#include <QPFD/Block.h>
_L_QPFD_BEGIN

namespace internal{
    template<int M, int N>
    using Mat = Eigen::Matrix<double,M,N>;

    // Left product
    Eigen::Matrix<double,3,3> getCrossProductMatrix(const double &x, const double &y, const double &z)
    {
        Eigen::Matrix<double,3,3> res;
        res << 0, -z, y, z, 0, -x, -y, x, 0;
        return res;
    }

    // Mapping euler angle velocity to rotation velocity in inertia frame
    // [wx,wy,wz]' = J*[rx,ry,rz]';
    Eigen::Matrix<double,3,3> getRotationJacobian(const double &rx, const double &ry, const double &rz)
    {
        Eigen::Matrix<double,3,3> J;
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
    }

    // Get mapping matrix from contact wrench to sum force and torque in inertia frame
    Eigen::Matrix<double, 6, 12> getMappingMatrix(const double BodyPos[3], const double FootPosL[3], const double FootPosR[3])
    {
        Eigen::Matrix<double, 6, 12> Res;
        Res.setZero();
        Res.block<3, 3>(0, 0).setIdentity();
        Res.block<3, 3>(0, 3).setIdentity();
        Res.block<3, 3>(3, 6).setIdentity();
        Res.block<3, 3>(3, 9).setIdentity();
        Res.block<3, 3>(3, 0) = getCrossProductMatrix(
            FootPosL[0] - BodyPos[0],
            FootPosL[1] - BodyPos[1],
            FootPosL[2] - BodyPos[2]);
        Res.block<3, 3>(3, 3) = getCrossProductMatrix(
            FootPosR[0] - BodyPos[0],
            FootPosR[1] - BodyPos[1],
            FootPosR[2] - BodyPos[2]);
        return Res;
    }
}

void Block::calRefAcc()
{
    double T = 0.004;
    auto &in = this->DataInput;
    this->LastOmg = this->RefOmg;

    // -- Linear acceleration feedforward
    for (int i = 0; i < 3; i++)
    {
        this->RefAcc(i) = in.LocalRefBodyAcc[i];
    }
    this->RefAcc(2) += 9.8;
    // -- Angular acceleration feedforward
    auto J = internal::getRotationJacobian(in.LocalRefBodyAng[0],in.LocalRefBodyAng[1],in.LocalRefBodyAng[2]);
    this->RefOmg = J*Eigen::Vector3d(in.LocalRefBodyAngularVel);
    this->RefAcc.bottomRows(3) = (this->RefOmg-this->LastOmg)/T;
}

_L_QPFD_END