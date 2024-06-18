#pragma once
#include <LBlocks/LBlocks.hpp>
// #include <Tool/RobotParameters.h>
#include <tools/Base.h>
// #include <DCM_Walk/Block.h>
#include <tools/DataType.h>

#define _S_DYN_FF_BEGIN _THESIS_TOOL_BEGIN namespace simple_dyn_ff{
#define _S_DYN_FF_END   _THESIS_TOOL_END }

_S_DYN_FF_BEGIN

struct Input{
    // const DCM_WalkPlanner::Output * pOutDCM;
    const double *pRefZMP;
    const double *pRefBody;
    const int *pSupFlag;
};

using Output = type::DynamicsReference;

class Block:public blocks::LBlock<Input,Output>
{
protected:
    math::Vec3 CoM_Bias;
    void calLocalRefZMP();
    void calWorldRefFT();
public:
    Block();
    int run();
    inline void setCoM_Bias(const math::Vec3 &_CoM_Bias){this->CoM_Bias=_CoM_Bias;};
};

_S_DYN_FF_END