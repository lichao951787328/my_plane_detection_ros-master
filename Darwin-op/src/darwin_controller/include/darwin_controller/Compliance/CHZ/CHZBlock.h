#include <LBlocks/LBlocks.hpp>
#include <LBlocks/LLog.hpp>

#include <tools/StateFlag.h>
#include <LGui/LGui.h>

namespace compliance{namespace chz{
    class Input
    {
    public:
    // 必需：
        // PG
        const double * dLAnkPosPG; // w下规划左踝
        const double * dRAnkPosPG; // w下规划右踝
        const double * dLFootFPG; // 规划左脚力
        const double * dRFootFPG; // 规划右脚力
        const int * nSupleg; // 规划支撑信号
        // sens
        const double * dLFootFSens; // 实际左脚力
        const double * dRFootFSens; // 实际右脚力

        const bool * pControlStartFlag;
        lee::blocks::LLog<> * pLogger;
    };

    class Output
    {
    public:
        double L_FootPosL[3]; // w下下发左踝位置
        double L_FootPosR[3]; // w下下发右踝位置
    };

    class ChzLandBlock:public lee::blocks::LBlock<Input, Output>
    {
    public:
        int init();
        int run();
        int clear();
        int log();
        int print();
        inline void setGUIFlag(const ljh::tools::GUIStateFlag &_flag){this->GUIFlag = _flag;};
        inline void setTssp (const double &_Tssp){this->Tssp = _Tssp * 0.80;};
		  char supleg, supleglast;
		  double tnow;
		  double Tssp;
    protected:
        ljh::tools::GUIStateFlag GUIFlag;
    };
}}
