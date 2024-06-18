#pragma once
#include <LBlocks/LBlocks.hpp>
#include <LSimpleRoboticsMath/Biped.hh>
#include <deque>
#include <tools/StateFlag.h>
#define _NET_PATH_BEGIN namespace lee{namespace path{namespace net_path{
#define _NET_PATH_END }}}

_NET_PATH_BEGIN

struct Input{
    const int *KeyPress;
    const double *BodyPos;
    const double *BodyAng;
    const double *FootPosL;
    const double *FootAngL;
    const double *FootPosR;
    const double *FootAngR;
    const int    *SupFlag;
};
struct Output{
    std::deque<math::biped::FootholdType> FootholdList;
};
class Block:public blocks::LBlock<Input,Output>
{
protected:
    char SendChar[2];
    double StepTime;
    ljh::tools::GUIStateFlag GUIFlag;

public:
    Block();
    ~Block();
    int init();
    int run();
    int print();
    int clear();

    void setIP(const char *_IP);
    void setPort(const int &_Port);
    bool getNetRunFlag();
    bool getDataRecvFlag();

    inline void setSendChar(const char &_Char1, const char &_Char2)
    {
        this->SendChar[0] = _Char1;
        this->SendChar[1] = _Char2;
    };
    void setPlotFlag(const bool &_Flag);
    inline void setStepTime(const double &_StepTime){this->StepTime = _StepTime;};
    inline void setGUIFlag(const ljh::tools::GUIStateFlag &_flag){this->GUIFlag = _flag;};
};

_NET_PATH_END