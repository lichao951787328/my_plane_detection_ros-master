#pragma once
#include "BHR_ControlFrame.h"
#include <tools/StateFlag.h>
#include <LGui/LGui.h>
_BHR_CF_BEGIN

namespace _youyi_walk_basic{

/* Define the Block Name-order map
 * which should be in accordance with the order int ini() in the Core.cpp
 * Once the order in int ini() is modified or New block added, the enum should be changed too.
 * this enum is set for calling -non-virtual- function in different block, because you need 
 * to clarify the detailed type(order) of the subblock to use the non-virtual functions by 
 * the getSubBlock function in LBlock.
*/
enum class YouYiControlFrameBlockOrder
{
    BlockKF,
    BlockSwtich,
    BlockDCM,
    BlockW2L,
    BlockCHZ,
    BlockQPFD,
    BlockDCC,
    BlockIK
};

class YOUYIControlFrame:public Frame
{
protected:
    int *SupFlag;
    ljh::tools::GUIStateFlag GUIFlag;
    

public:
    inline YOUYIControlFrame(){};
    inline YOUYIControlFrame(const double &_Ts):Frame(_Ts){};
    YOUYIControlFrame(const YOUYIControlFrame & other);
    YOUYIControlFrame & operator=(const YOUYIControlFrame & other);
    int init();
    int run();
    int log();
    int clear();
    int print();
    void setParameters();
    inline void setGUIFlag(const ljh::tools::GUIStateFlag &_flag){this->GUIFlag = _flag;};
};


}
_BHR_CF_End
