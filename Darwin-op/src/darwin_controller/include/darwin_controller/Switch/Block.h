#include <LBlocks/LBlocks.hpp>
#include <LSimpleRoboticsMath/Biped.hh>
#include <deque>
#include <tools/StateFlag.h>
#define _SWITCH_BEGIN namespace lee{namespace path{namespace _switch{
#define _SWITCH_END }}}

_SWITCH_BEGIN

enum{REMOTE, PERCEPTION};
constexpr int STAND = 0;
struct Input{
    const int *PressKey;
    const int *WalkState;
    const int *SupFlag;
    const double *BodyPos;
    const double *BodyAng;
    const double *RefFootPosL;
    const double *RefFootAngL;
    const double *RefFootPosR;
    const double *RefFootAngR;
    std::deque<math::biped::FootholdType> **ppFootholdList;
};
struct Output{
    int PathState;
    int RemoteKey;
};

class Block:public blocks::LBlock<Input,Output>
{
protected:
    int *pRemoteKey;
    std::deque<math::biped::FootholdType> *pFootholdListRemote, *pFootholdListPerception;
    ljh::tools::GUIStateFlag GUIFlag;
public:
    Block();
    int init();
    int run();
    int print();
    int log();
    int clear();
    inline void setGUIFlag(const ljh::tools::GUIStateFlag &_flag){this->GUIFlag = _flag;};
};

_SWITCH_END