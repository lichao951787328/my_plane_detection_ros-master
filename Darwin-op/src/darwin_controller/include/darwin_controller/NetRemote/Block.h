#pragma once
#include <LBlocks/LBlocks.hpp>
#include <LGui/LGui.h>
#include <tools/StateFlag.h>
#define _NET_REMOTE_BEGIN   namespace lee{namespace path{namespace net_remote{
#define _NET_REMOTE_END }}}

_NET_REMOTE_BEGIN

struct Input{
    const int *PressKey;
};

struct Output{
    double *LinearVel;
    double *AngularVel;
    int *NetKey;
    
    bool RunFlag;
};

class Block:public blocks::LBlock<Input,Output>
{
protected:
    ljh::tools::GUIStateFlag GUIFlag;
public:
    Block();
    ~Block();
    int init();
    int run();
    int clear();
    int print();
    inline void setGUIFlag(const ljh::tools::GUIStateFlag &_flag){this->GUIFlag = _flag;};
};
_NET_REMOTE_END