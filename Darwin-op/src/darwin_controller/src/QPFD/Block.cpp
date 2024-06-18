#include <QPFD/Block.h>
#include <iostream>
#include <LBlocks/LLog.hpp>

_L_QPFD_BEGIN

Block::Block()
{
    this->UseGUI = false;
    this->setModelParameters(55.0, {4.0, 4.0, 0.9}, {0, 0, 0});
    this->setOutput({
        &this->QP.Solver.x(LFX),
        &this->QP.Solver.x(RFX),
        &this->QP.Solver.x(LTX),
        &this->QP.Solver.x(RTX)
    });
    this->QP.CostSeq = 0;
    this->QP.IeqConsSeq = 0;
    this->QP.Solver.setProblem(VAR_NUM,0,12+4+8,30);
    this->QP.Solver.Ai.setZero();
    this->QP.Solver.bi.setZero();
    this->QP.Solver.A.setZero();
    this->QP.Solver.b.setZero();
    this->QP.Solver.x.setZero();
    this->QP.Flag = 0;

    this->RefAcc.setZero();
    this->RefOmg.setZero();
    this->LastOmg.setZero();
    
    this->Params.ContactWeight = 1e-3;
    this->Params.NoContactWeight = 1e3;
    this->Params.SmoothWeight = 1e-4;
    
    std::cout<<"Create Block: QP based force distribution"<<std::endl;
}
int Block::init()
{
    return 0;
}
int Block::run()
{
    this->CoM = this->CoM_Bias + Eigen::Vector3d(this->DataInput.LocalRefBodyPos);
    this->solve();
    return 0;
}

void Block::setModelParameters(
        const double &Mass,
        const Eigen::Vector3d &InertiaVector,
        const Eigen::Vector3d &CoM_Bias)
{
    this->CoM_Bias = CoM_Bias; // {0.015, 0.0, 0.16};
    this->Phy.Mass = Mass;
    // double r2 = 0.3*0.3;
    // this->Phy.Inertia[0] = 2.0/5.0*this->Phy.Mass*r2/10.0;
    // this->Phy.Inertia[1] = 2.0/5.0*this->Phy.Mass*r2/20.0;
    // this->Phy.Inertia[2] = 2.0/5.0*this->Phy.Mass*r2/20.0;
    this->Phy.Inertia[0] = InertiaVector(0);
    this->Phy.Inertia[1] = InertiaVector(1);
    this->Phy.Inertia[2] = InertiaVector(2);

    this->Phy.InertiaMatrix.setZero();
    for(int i=0;i<3;i++)
    {
        this->Phy.InertiaMatrix(i,i) = this->Phy.Mass;
        this->Phy.InertiaMatrix(i+3,i+3) = this->Phy.Inertia[i];
    }
}

double SingleZMPL[2] = {0,0};
double SingleZMPR[2] = {0,0};
int Block::print()
{
    if(this->UseGUI)
    {
        this->printLGui();
        return 1;
    }
    auto &in = this->DataInput;
    auto &out = this->DataOutput;
    std::cout<<"[QP] "<<this->QP.Flag<<"  "<<std::endl;
    std::cout<<"L_Body_Pos: "
    <<in.LocalRefBodyPos[0]<<", "
    <<in.LocalRefBodyPos[1]<<", "
    <<in.LocalRefBodyPos[2]<<", "
    <<std::endl
    <<"Force: "
    <<out.LocalRefFootForceL[0]<<", "
    <<out.LocalRefFootForceL[1]<<", "
    <<out.LocalRefFootForceL[2]<<", "
    <<out.LocalRefFootForceR[0]<<", "
    <<out.LocalRefFootForceR[1]<<", "
    <<out.LocalRefFootForceR[2]<<", "
    <<std::endl
    <<"Torque: "
    <<out.LocalRefFootTorqueL[0]<<", "
    <<out.LocalRefFootTorqueL[1]<<", "
    <<out.LocalRefFootTorqueL[2]<<", "
    <<out.LocalRefFootTorqueR[0]<<", "
    <<out.LocalRefFootTorqueR[1]<<", "
    <<out.LocalRefFootTorqueR[2]<<", "
    <<std::endl;

    std::cout<<"ZMP: "<<SingleZMPL[0]<<", "<<SingleZMPL[1]<<std::endl;
    std::cout<<"   : "<<SingleZMPR[0]<<", "<<SingleZMPR[1]<<std::endl;
    return 0;
}

blocks::LLog<> Logger;
int Block::log()
{
    auto &out = this->DataOutput;
    Logger.startLog();
    Logger.addLog(out.LocalRefFootForceL[0],"RefForceLX");
    Logger.addLog(out.LocalRefFootForceL[1],"RefForceLY");
    Logger.addLog(out.LocalRefFootForceL[2],"RefForceLZ");
    Logger.addLog(out.LocalRefFootForceR[0],"RefForceRX");
    Logger.addLog(out.LocalRefFootForceR[1],"RefForceRY");
    Logger.addLog(out.LocalRefFootForceR[2],"RefForceRZ");

    Logger.addLog(out.LocalRefFootTorqueL[0],"RefTorqueLX");
    Logger.addLog(out.LocalRefFootTorqueL[1],"RefTorqueLY");
    Logger.addLog(out.LocalRefFootTorqueL[2],"RefTorqueLZ");
    Logger.addLog(out.LocalRefFootTorqueR[0],"RefTorqueRX");
    Logger.addLog(out.LocalRefFootTorqueR[1],"RefTorqueRY");
    Logger.addLog(out.LocalRefFootTorqueR[2],"RefTorqueRZ");

    auto &in = this->DataInput;
    Logger.addLog(in.LocalRefBodyAcc[0],"RefBodyAccX");
    Logger.addLog(in.LocalRefBodyAcc[1],"RefBodyAccY");
    Logger.addLog(in.LocalRefBodyAcc[2],"RefBodyAccZ");
    Logger.addLog(this->RefAcc(3),"RefBodyAngAccX");
    Logger.addLog(this->RefAcc(4),"RefBodyAngAccY");
    Logger.addLog(this->RefAcc(5),"RefBodyAngAccZ");

    Logger.addLog(in.LocalRefFootPosL[0],"RefFootPosLX");
    Logger.addLog(in.LocalRefFootPosL[1],"RefFootPosLY");
    Logger.addLog(in.LocalRefFootPosL[2],"RefFootPosLZ");
    Logger.addLog(in.LocalRefFootPosR[0],"RefFootPosRX");
    Logger.addLog(in.LocalRefFootPosR[1],"RefFootPosRY");
    Logger.addLog(in.LocalRefFootPosR[2],"RefFootPosRZ");

    Logger.addLog(*in.SupFlag,"SupFlag");

    Logger.addLog(this->QP.Flag,"QPFlag");

    auto &ltx = out.LocalRefFootTorqueL[0];
    auto &lty = out.LocalRefFootTorqueL[1];
    auto &lfz = out.LocalRefFootForceL[2];
    
    auto &rtx = out.LocalRefFootTorqueR[0];
    auto &rty = out.LocalRefFootTorqueR[1];
    auto &rfz = out.LocalRefFootForceR[2];

    if(lfz>1.0)
    {
        SingleZMPL[0] = lty/lfz;
        SingleZMPL[1] =-ltx/lfz;
    }
    else
    {
        SingleZMPL[0] = 0;
        SingleZMPL[1] = 0;
    }

    if(rfz>1.0)
    {
        SingleZMPR[0] = rty/rfz;
        SingleZMPR[1] =-rtx/rfz;
    }
    else
    {
        SingleZMPR[0] = 0;
        SingleZMPR[1] = 0;
    }
    Logger.addLog(SingleZMPL[0],"ZMP_LX");
    Logger.addLog(SingleZMPL[1],"ZMP_LY");
    Logger.addLog(SingleZMPR[0],"ZMP_RX");
    Logger.addLog(SingleZMPR[1],"ZMP_RY");


    return 0;
}
int Block::clear()
{
    if(Logger.getNameList().size()==0)
        return -1;
    Logger.saveLog("qpfd.dat");
    return 0;
}
_L_QPFD_END