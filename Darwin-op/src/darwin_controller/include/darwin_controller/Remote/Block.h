// Remote/Block.h
// lee, hexb66@bit.edu.cn
// Mar. 20, 2022
#pragma once
#include "Core.hpp"
#include <LBlocks/LBlocks.hpp>

#include <LGui/LGui.h>
#include <tools/StateFlag.h>
_REMOTE_BEGIN
enum _REMOTE_STATE{REMOTE_OFF, REMOTE_ON};
struct Input{
    const int *PressKey;
    const int *SupFlag; 
    const double *RefFootPosL;
    const double *RefFootAngL;
    const double *RefFootPosR;
    const double *RefFootAngR;
};
struct Output{
    std::deque<FootholdType> *pFootholdList;
    
    int RemoteKey;
};

class Block:public lee::blocks::LBlock<Input, Output>, public Remote<>
{
protected:
    int StateFlag;
    ljh::tools::GUIStateFlag GUIFlag;
    void calCmdVel();
    void stopWalk();

public:
    Block();
    int init();
    int run();
    int print();
    int clear();

    inline void setStepTime(const double& _StepTime)
    {
        for(auto &i:this->StepList)
            i.StepTime = _StepTime;
        for(auto &i:this->FootholdList)
            i.StepTime = _StepTime;
        this->LastFoothold.StepTime = _StepTime;
    };

    inline void setGUIFlag(const ljh::tools::GUIStateFlag &_flag){this->GUIFlag = _flag;};
};

_REMOTE_END