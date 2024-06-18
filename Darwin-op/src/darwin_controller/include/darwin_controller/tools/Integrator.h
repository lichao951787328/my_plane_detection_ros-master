#pragma once
#include "Base.h"
#include "Math.hpp"
#include <LBlocks/LBlocks.hpp>

#define _INTEGRATOR_BEGIN _THESIS_TOOL_BEGIN namespace integrator{
#define _INTEGRATOR_END }_THESIS_TOOL_END

_INTEGRATOR_BEGIN

using tools::math::Vec3;
using tools::math::State3;

struct Input{
    std::vector<const double *> AccList;
};
struct Output{
    State3 State;
};
class Block:public lee::blocks::LBlock<Input,Output>
{
protected:
    double Ts;
public:
    Block(const double &_Ts, const char *_Name="Anonymous");
    int init();
    int run();
    int print();
};
_INTEGRATOR_END