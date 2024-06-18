#include <LBlocks/LBlocks.hpp>
#include <LBlocks/LLog.hpp>
// #define USE_DCC_METHOD
#define USE_HONDA_METHOD
extern "C" {
#include "DccCon_lib/DCC_ConFrame.h"
#include "DccCon_lib/DCC_RunningCon.h"
}
#include <tools/StateFlag.h>
#include <LGui/LGui.h>
namespace compliance{namespace dcc{
    class Input
    {
    public:
    // 必需：
        // dccarm ======================================
        // arm
        const double * dLegLq; // 左腿3、4关节速度 
        const double * dLegRq; // 右腿3、4关节速度 
        // dccarm ======================================
        // PG
        const double * dBasePosPG; // w下规划质心位置
        const double * dBasedPosPG; // 规划质心速度
        const double * dBaseRotPG; // 规划上身姿态
        const double * dBasedRotPG; // 规划上身姿态角速度
        const double * dLAnkPosPG; // w下规划左踝位置
        const double * dLAnkRotPG; // 规划左踝姿态
        const double * dRAnkPosPG; // w下规划右踝位置
        const double * dRAnkRotPG; // 规划右踝姿态
        const double * dLFootFPG; // 规划左脚力
        const double * dRFootFPG; // 规划右脚力
        const double * dLFootTPG; // 规划左脚力矩
        const double * dRFootTPG; // 规划右脚力矩
        const int * nSupleg; // 规划支撑信号
        // sens
        const double * dBaseRotSens; // 实际上身姿态
        const double * dLFootFSens; // 实际左脚力
        const double * dRFootFSens; // 实际右脚力
        const double * dLFootTSens; // 实际左脚力矩
        const double * dRFootTSens; // 实际右脚力矩
    // 可选：
        // PG
        const double * dBaseddPosPG; // 规划质心加速度
        const double * dZMPwPG; // w下规划ZMP
        const double * dZMPbPG; // b下规划ZMP
        // sens
        const double * dZMPwSens; // w下实际ZMP
        const double * dZMPbSens; // b下实际ZMP

        const bool * pControlStartFlag;
        lee::blocks::LLog<> *pLogger;
    };

    class Output
    {
    public:
        // dccarm ======================================
        double *dArmLq;       // 手臂1关节角度 
        double *dArmRq;       // 手臂1关节角度 
        // dccarm ======================================
        double dBasePosCmd[3]; // w下下发质心位置
        double dBaseRotCmd[3]; // 下发上身姿态
        double dLAnkPosCmd[3]; // w下下发左踝位置
        double dLAnkRotCmd[3]; // 下发左踝姿态
        double dRAnkPosCmd[3]; // w下下发右踝位置
        double dRAnkRotCmd[3]; // 下发右踝姿态
    };

    class Block:public lee::blocks::LBlock<Input, Output>
    {
    public:
        int init();
        int run();
        int clear();
        int log();
        int print();
        inline void setGUIFlag(const ljh::tools::GUIStateFlag &_flag){this->GUIFlag = _flag;};
    protected:
		double dTLagZMP;
		double dTLagIMU;
		double dTLagdIMU;
		double dTLagFrc;
		double dTLagTrq;
		void fnvStateInit(dccRobotState * stStateName);
		void fnvGetStatePG();
		void fnvGetStateSens();
		void fnvGetConVal();
		void fnvAddConVal();

        ljh::tools::GUIStateFlag GUIFlag;
    };
}}
