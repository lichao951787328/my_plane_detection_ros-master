#pragma once
#include <LBlocks/LBlocks.hpp>
#include <tools/StateFlag.h>
#include <LGui/LGui.h>
namespace lee{namespace posture_kf{
    class Input
    {
    public:
        double *IMU_Ang;
    };

    class Output
    {
    public:
        double *EstimatedAng;
    };

    class Block:public lee::blocks::LBlock<Input,Output>
    {
    public:
        Block(const double& _Ts);
        int init();
        int run();
        int print();
        int clear();
        int log();
        inline void setGUIFlag(const ljh::tools::GUIStateFlag &_flag){this->GUIFlag = _flag;};
    protected:
        double EstimatedAng[3];
        double Ts;
        ljh::tools::GUIStateFlag GUIFlag;
    };
}}
