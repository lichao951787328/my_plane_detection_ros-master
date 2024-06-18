#pragma once
#include "Base.h"
#include <LBlocks/LBlocks.hpp>
#include <LBlocks/LLog.hpp>
#include <LSimpleRoboticsMath/Kinematics.hh>
#define _TRA_SUM_BEGIN _THESIS_TOOL_BEGIN namespace tra_sum{
#define _TRA_SUM_END _THESIS_TOOL_END}

_TRA_SUM_BEGIN

using lee::math::kinematics::Vec3;
struct Input{
    std::vector<const double *> 
    BodyPos,  BodyAng, 
    FootPosL, FootAngL, 
    FootPosR, FootAngR;
};
struct Output{
    Vec3 BodyPos,  BodyAng;
    Vec3 FootPosL, FootAngL;
    Vec3 FootPosR, FootAngR;
};

class Block:public lee::blocks::LBlock<Input,Output>
{
protected:
    std::vector<std::vector<const double *> *> InList;
    std::vector<Vec3 *> OutList;
    blocks::LLog<> *pExtLogger;
public:
    Block();
    int init();
    int run();
    int print();
    int log();
    inline void setLogger(blocks::LLog<> *pLogger){this->pExtLogger = pLogger;};
};
_TRA_SUM_END