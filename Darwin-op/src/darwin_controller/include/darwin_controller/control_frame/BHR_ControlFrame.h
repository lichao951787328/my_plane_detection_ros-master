#pragma once
#include <LBlocks/LBlocks.hpp>
#include <LBlocks/LLog.hpp>
#include <tools/StateFlag.h>
#define _BHR_CF_BEGIN namespace BHR{namespace YouYi_ControlFrame{
#define _BHR_CF_End }}

_BHR_CF_BEGIN

class Input
{
public:
    // ROBOTIQ 2F Gripper (Only care about the actual position)
    const uint8_t * RealGripperPosL;
    const uint8_t * RealGripperPosR;

    // Joint 
    const double * RealLegJointL;
    const double * RealLegJointR;
    const double * RealArmJointL;
    const double * RealArmJointR;
    
    // ForceSensor Foot
    const double * FSFootForceL;
    const double * FSFootForceR;
    const double * FSFootTorqueL;
    const double * FSFootTorqueR;
    // ForceSensor Hand
    const double * FSWristForceL;
    const double * FSWristForceR;
    const double * FSWristTorqueL;
    const double * FSWristTorqueR;
    // ForceSensor Hip
    const double * FSHipForceL;
    const double * FSHipForceR;
    const double * FSHipTorqueL;
    const double * FSHipTorqueR;

    // ForceSensor Joint
    const double * JFSKneeForceL;
    const double * JFSKneeForceR;
    const double * JFSKneeTorqueL;
    const double * JFSKneeTorqueR;

    const double * JFSAnkleForceL;
    const double * JFSAnkleForceR;
    const double * JFSAnkleTorqueL;
    const double * JFSAnkleTorqueR;

    // IMU
    const double * RealAng;
    const double * RealAcc;
    const double * RealOmg;

    // KeyPress
    const int * PressKey;

    // Logger
    lee::blocks::LLog<> *pLogger;
};

/* for Position/Torque control
 * When motor is in Position Control, load the output of reference pos of Joints
 * When motor is in Torque Control, load the output of reference torque of Joints 
*/
class Output
{
public:
    // Leg & Arm Joint Ref Pos
    double * RefLegJointL;
    double * RefLegJointR;
    double * RefArmJointL;
    double * RefArmJointR;
    // Leg & Arm Joint Ref Troque
    double * RefTorqueLegJointL;
    double * RefTorqueLegjointR;
    double * RefTorqueArmJointL;
    double * RefTorqueArmJointR;
    // For ROBOTIQ 2F Grippers
    uint8_t * RefGripperPosL;
    uint8_t * RefGripperPosR;
    // For JointControl Specification & Switching: PosCntl<->TorCntl
    ljh::tools::MotorControlTypeSeries<24>* JointCntlFlag;
};

class Frame:public lee::blocks::LBlock<Input,Output>
{
protected:
    double RevisedAng[3];
    double LocalRealBodyOmega[3];
    //RobotKinematicsParameters Config;
    double Time, Ts;

    double RealBodyZMP[2];
    double RefFootForce[2][3];
    double RefFootTorque[2][3];
    double EstFootForce[2][3];
    bool ControlStartFlag;
    lee::blocks::LLog<> *pLogger;
    //lee::blocks::LLog<double> Logger;

    // Choose Control Type
    int ControlStateFlag;
    
public:
    Frame(double _Ts = 0.001);
    Frame(const Frame & other);
    Frame & operator=(const Frame & other);
    ~Frame(){};
    int run();
    int init();
    int clear();
    int print();
    int log();

    void switchToPositionControl(const ljh::tools::ControlType &_Type){this->ControlStateFlag = ljh::tools::ControlType::PositionControl;};
    void switchToTorqueControl(const ljh::tools::ControlType &_Type){this->ControlStateFlag = ljh::tools::ControlType::TorqueControl;};
    const int& getControlStateFlag(){return this->ControlStateFlag;};
protected:
    void calRealBodyZMP(const double *_BodyAng);
    void calRefForce(const int &_SupFlag);
    void controlPosture(const double *_BodyAng);
    void estFootForce();
    void initFootForceEstimator();

    // Tool functions
    void plotData(const char *_Str);

    void setTs(const double &_Ts){this->Ts = _Ts;};
    
    // MotorControlType functions
    // void setJointCntlType(ljh::tools::MotorControlType *_JointCntlType)
    // {this->JointCntlType = _JointCntlType;};
};

_BHR_CF_End