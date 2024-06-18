// QPFD/Block.h
// QP based force distribution
// lee, hexb66@bit.edu.cn
// Apr. 18, 2022
#pragma once
#include "Base.h"
#include <LBlocks/LBlocks.hpp>
_L_QPFD_BEGIN

struct Input
{
    const double *LocalRefBodyPos;
    const double *LocalRefBodyVel;
    const double *LocalRefBodyAcc;
    const double *LocalRefBodyAng;
    const double *LocalRefBodyAngularVel;
    // const double *LocalRefBodyAngularAcc;

    const double *LocalRefFootPosL;
    const double *LocalRefFootPosR;

    const int    *SupFlag;
};
struct Output
{
    double *LocalRefFootForceL;
    double *LocalRefFootForceR;
    double *LocalRefFootTorqueL;
    double *LocalRefFootTorqueR;
};

class Block:public blocks::LBlock<Input,Output>
{
protected:
    struct {
        float SmoothWeight, ContactWeight, NoContactWeight;
    } Params;
    bool UseGUI;
    Eigen::Matrix<double,6,1> RefAcc;
    Eigen::Matrix<double,3,1> RefOmg, LastOmg;
    Eigen::Vector3d CoM_Bias, CoM;
    struct{
        eigen_qp::LEigenQP<VAR_NUM> Solver;
        Eigen::Matrix<double,VAR_NUM,1> LastResult;
        int CostSeq, IeqConsSeq, Flag;
    } QP;
    struct{
        double Mass, Inertia[3];
        Eigen::Matrix<double,6,6> InertiaMatrix;
    } Phy;

    template<typename TypeA, typename TypeB> 
    void addCost(TypeA &A, TypeB &b, const double *Weight)
    {
        for(int i=0;i<A.rows();i++,this->QP.CostSeq++)
        {
            this->QP.Solver.A.row(this->QP.CostSeq) = A.row(i)*Weight[i];
            this->QP.Solver.b(this->QP.CostSeq) = b(i)*Weight[i];
        }
    };

    template<typename TypeA, typename TypeB>
    void addIEqConstraint(TypeA &A, const TypeB &b)
    {
        for(int i=0;i<A.rows();i++,this->QP.IeqConsSeq++)
        {
            this->QP.Solver.Ai.row(this->QP.IeqConsSeq) = A.row(i);
            this->QP.Solver.bi(this->QP.IeqConsSeq) = b(i);
        }
    };

    void solve();
    void calRefAcc();

    void printLGui();

public:
    Block();
    int init();
    int run();
    int print();
    int clear();
    int log();

    void setModelParameters(
        const double &Mass,
        const Eigen::Vector3d &InertiaVector,
        const Eigen::Vector3d &CoM_Bias
    );
    inline void setGuiFlag(const bool &Flag){this->UseGUI = Flag;};
};

_L_QPFD_END