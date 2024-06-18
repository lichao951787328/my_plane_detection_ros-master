#include <tools/SimpleDynamicsFeedForward.h>
#include <tools/RobotParameters.h>
#include <LSimpleRoboticsMath/Biped.hh>
#include <iostream>

_S_DYN_FF_BEGIN

Block::Block()
{
    this->BlockName = "Simple Dyn FF";
    this->DataOutput.LocalZMP.setZero();
    this->CoM_Bias.setZero();

    std::cout<<"Create Block: "<<this->BlockName<<std::endl;
}

int Block::run()
{
    this->calLocalRefZMP();
    this->calWorldRefFT();
    return 0;
}

void Block::calLocalRefZMP()
{
    using math::Vec3;
    // auto &pRef = this->DataInput.pOutDCM;
    // this->DataOutput.LocalZMP = Vec3{pRef->RefZMP} - Vec3{pRef->RefBodyPos};
    this->DataOutput.LocalZMP = Vec3{this->DataInput.pRefZMP} - Vec3{this->DataInput.pRefBody} - this->CoM_Bias;
}

void Block::calWorldRefFT()
{
    auto &SupFlag = *this->DataInput.pSupFlag;
    switch (SupFlag)
    {
    // case dcm::L_SUP:
    case lee::math::biped::L_SUP:
        this->DataOutput.FT_L.Force(2) = tools::RobotWeight;
        this->DataOutput.FT_R.Force(2) = 0.0;
        break;
    // case dcm::R_SUP:
    case lee::math::biped::R_SUP:
        this->DataOutput.FT_R.Force(2) = tools::RobotWeight;
        this->DataOutput.FT_L.Force(2) = 0.0;
        break;
    // case dcm::DS_SUP:
    case lee::math::biped::DS_SUP:
        this->DataOutput.FT_L.Force(2) = 0.5*tools::RobotWeight;
        this->DataOutput.FT_R.Force(2) = 0.5*tools::RobotWeight;
        break;
    default:
        break;
    }
}

_S_DYN_FF_END