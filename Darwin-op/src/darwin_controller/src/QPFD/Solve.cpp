#include <QPFD/Block.h>
#include <LSimpleRoboticsMath/Biped.hh>
_L_QPFD_BEGIN

namespace internal{
    template<int M, int N>
    using Mat = Eigen::Matrix<double,M,N>;
    // Left product
    Eigen::Matrix<double,3,3> getCrossProductMatrix(const double &x, const double &y, const double &z);
    // Mapping euler angle velocity to rotation velocity in inertia frame
    // [wx,wy.wz]' = J*[rx,ry,rz]';
    Eigen::Matrix<double,3,3> getRotationJacobian(const double &rx, const double &ry, const double &rz);
    // Get mapping matrix from contact wrench to sum force and torque in inertia frame
    Eigen::Matrix<double, 6, 12> getMappingMatrix(const double BodyPos[3], const double FootPosL[3], const double FootPosR[3]);
}

void Block::solve()
{
    using math::biped::L_SUP;
    using math::biped::R_SUP;
    auto &qp = this->QP.Solver;
    auto &in = this->DataInput;
    this->QP.LastResult = qp.x;
    this->QP.CostSeq = 0;
    this->QP.IeqConsSeq = 0;

    auto MappintMatrix = internal::getMappingMatrix(this->CoM.data(), in.LocalRefFootPosL, in.LocalRefFootPosR);

    // --------- set cost ------------
    // Float base control
    // -- M*ddx = MappingMatrix*W
    this->calRefAcc();
    this->addCost(
        MappintMatrix,
        this->Phy.InertiaMatrix * this->RefAcc,
        internal::Mat<6, 1>(internal::Mat<6, 1>::Constant(1 * 1e3)).data());

    // Smooth optimized variables
    Eigen::Matrix<double, VAR_NUM, VAR_NUM> ItermMappingMatrix;
    ItermMappingMatrix.setIdentity();
    this->addCost(
        ItermMappingMatrix,
        this->QP.LastResult,
        internal::Mat<VAR_NUM, 1>(internal::Mat<VAR_NUM, 1>::Constant((double) this->Params.SmoothWeight)).data());

    // Small control input
    Eigen::Matrix<double, VAR_NUM, 1> ZeroInput;
    ZeroInput.setZero();
    Eigen::Matrix<double, VAR_NUM, 1> ContactRelatedWeight;
    ContactRelatedWeight.setConstant((double) this->Params.ContactWeight);
    if (*in.SupFlag == R_SUP)
    {
        ContactRelatedWeight.middleRows<3>(LFX).setConstant((double) this->Params.NoContactWeight);
        ContactRelatedWeight.middleRows<3>(LTX).setConstant((double) this->Params.NoContactWeight);
    }
    if (*in.SupFlag == L_SUP)
    {
        ContactRelatedWeight.middleRows<3>(RFX).setConstant((double) this->Params.NoContactWeight);
        ContactRelatedWeight.middleRows<3>(RTX).setConstant((double) this->Params.NoContactWeight);
    }
    this->addCost(ItermMappingMatrix, ZeroInput, ContactRelatedWeight.data());

    // --------- set ieq constraint --
    // {1} Friction limitation: fx < fz*u ==> fx - u*fz < 0
    //                      fx >-fz*u ==>-fx - u*fz < 0
    //                      tz < u*fz*R*2/3 ==> tz - u*R*2/3*fz < 0
    //                      tz >-u*fz*R*2/3 ==>-tz - u*R*2/3*fz < 0
    double _u = 0.8, _uM=_u*0.2;
    Eigen::Matrix<double, 12, VAR_NUM> FrictionMatrix;
    FrictionMatrix.setZero();
    FrictionMatrix(0, LFX) = 1.0; FrictionMatrix(0, LFZ) = -_u;  
    FrictionMatrix(1, LFX) =-1.0; FrictionMatrix(1, LFZ) = -_u;  
    FrictionMatrix(2, LFY) = 1.0; FrictionMatrix(2, LFZ) = -_u;  
    FrictionMatrix(3, LFY) =-1.0; FrictionMatrix(3, LFZ) = -_u;  
    FrictionMatrix(4, LTZ) = 1.0; FrictionMatrix(4, LFZ) = -_uM; 
    FrictionMatrix(5, LTZ) =-1.0; FrictionMatrix(5, LFZ) = -_uM; 
    FrictionMatrix(0+6, RFX) = 1.0; FrictionMatrix(0+6, RFZ) = -_u; 
    FrictionMatrix(1+6, RFX) =-1.0; FrictionMatrix(1+6, RFZ) = -_u; 
    FrictionMatrix(2+6, RFY) = 1.0; FrictionMatrix(2+6, RFZ) = -_u; 
    FrictionMatrix(3+6, RFY) =-1.0; FrictionMatrix(3+6, RFZ) = -_u; 
    FrictionMatrix(4+6, RTZ) = 1.0; FrictionMatrix(4+6, RFZ) = -_uM; 
    FrictionMatrix(5+6, RTZ) =-1.0; FrictionMatrix(5+6, RFZ) = -_uM; 

    this->addIEqConstraint(
        FrictionMatrix, Eigen::Matrix<double, 12, 1>::Zero());
    // {2} Fz > 0, Fz<1.5*Weight
    internal::Mat<2, VAR_NUM> FzLimitMatrixL, FzLimitMatrixR;
    FzLimitMatrixL.setZero();
    FzLimitMatrixR.setZero();
    FzLimitMatrixL(0, LFZ) = -1.0;
    FzLimitMatrixL(1, LFZ) =  1.0;
    FzLimitMatrixR(0, RFZ) = -1.0;
    FzLimitMatrixR(1, RFZ) =  1.0;
    this->addIEqConstraint(FzLimitMatrixL, Eigen::Vector2d({0.0, 1.5 * this->Phy.Mass * 9.8}));
    this->addIEqConstraint(FzLimitMatrixR, Eigen::Vector2d({0.0, 1.5 * this->Phy.Mass * 9.8}));
    // {3} ZMP limitation:
    // tx - fz * zmp_max_y < 0.0, tx - fz * zmp_min_y > 0.0
    // -ty - fz * zmp_max_x < 0.0, -ty - fz * zmp_min_x > 0.0
    // ==> tx-fz*zmp_max_y < 0.0
    //    -tx+fz*zmp_min_y < 0.0
    //    -ty-fz*zmp_max_x < 0.0
    //     ty+fz*zmp_min_x < 0.0
    double LimitZMP_X[2] = {-0.1, 0.1};
    double LimitZMP_Y[2] = {-0.05, 0.05};
    Eigen::Matrix<double, 8, VAR_NUM> ZMPMatrix;
    ZMPMatrix.setZero();
    ZMPMatrix(0, LTX) =  1.0; ZMPMatrix(0, LFZ) = -LimitZMP_Y[1];
    ZMPMatrix(1, LTX) = -1.0; ZMPMatrix(1, LFZ) =  LimitZMP_Y[0];
    ZMPMatrix(2, LTY) = -1.0; ZMPMatrix(2, LFZ) = -LimitZMP_X[1];
    ZMPMatrix(3, LTY) =  1.0; ZMPMatrix(3, LFZ) =  LimitZMP_X[0];

    ZMPMatrix(4, RTX) =  1.0; ZMPMatrix(4, RFZ) = -LimitZMP_Y[1];
    ZMPMatrix(5, RTX) = -1.0; ZMPMatrix(5, RFZ) =  LimitZMP_Y[0];
    ZMPMatrix(6, RTY) = -1.0; ZMPMatrix(6, RFZ) = -LimitZMP_X[1];
    ZMPMatrix(7, RTY) =  1.0; ZMPMatrix(7, RFZ) =  LimitZMP_X[0];
    
    this->addIEqConstraint(
        ZMPMatrix, Eigen::Matrix<double, 8, 1>::Zero());

    // --------- set eq constraint ---
    // None

    // --------- solve ---------------
    qp.updateHf();
    this->QP.Flag = qp.getResult();
    if(!this->QP.Flag)
    {
        qp.x = this->QP.LastResult;
    }
}

_L_QPFD_END