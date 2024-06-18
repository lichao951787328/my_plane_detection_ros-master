#ifndef CONTROL_INTERFACE_H_
#define CONTROL_INTERFACE_H_
#include <vector>
#include <control_frame/ControlFrameFrictionTest.h>
#include <control_frame/ControlFrameWalkBasic.h>
using namespace std;

class input_interface
{
public:
    double real_joint_left_leg[6];
    double real_joint_right_leg[6];
    double real_joint_left_arm[6];
    double real_joint_right_arm[6];

    double FSForceLeft[3], FSTorqueLeft[3];
    double FSForceRight[3], FSTorqueRight[3];
    double NonUsageFSForce[6][3];
    double NonUsageFSTorque[6][3];
    double Acc[3], Ang[3], Omg[3];
};

class control_interface
{
private:

// input interface
    // gripper
    uint8_t RealGripperPosL;
    uint8_t RealGripperPosR;
    // joint
    double real_joint_left_leg[6];
    double real_joint_right_leg[6];
    double real_joint_left_arm[6];
    double real_joint_right_arm[6];
    // force sensor foot
    double FSForceLeftFoot[3];
    double FSForceRightFoot[3];
    double FSTorqueLeftFoot[3];
    double FSTorqueRightFoot[3];
    // force sensor hand
    double FSForceLeftWrist[3];
    double FSForceRightWrist[3];
    double FSTorqueLeftWrist[3];
    double FSTorqueRightWrist[3];
    // force sensor hip
    double FSForceLeftHip[3];
    double FSForceRightHip[3];
    double FSTorqueLeftHip[3];
    double FSTorqueRightHip[3];

    // force senor joint
    double JFSKneeForceL[3];
    double JFSKneeForceR[3];
    double JFSKneeTorqueL[3];
    double JFSKneeTorqueR[3];

    double JFSAnkleForceL[3];
    double JFSAnkleForceR[3];
    double JFSAnkleTorqueL[3];
    double JFSAnkleTorqueR[3];

    // imu
    double Ang[3], Acc[3], Omg[3];

    int key;
    lee::blocks::LLog<double> Logger;
// intput end

// output
    // leg and arm ref pos
    double ref_joint_left_leg[6];
    double ref_joint_right_leg[6];
    double ref_joint_left_arm[4];
    double ref_joint_right_arm[4];

    // leg and arm ref troque
    double RefTorqueLegJointL[6];
    double RefTorqueLegjointR[6];
    double RefTorqueArmJointL[4];
    double RefTorqueArmJointR[4];

    // gripper
    uint8_t RefGripperPosL;
    uint8_t RefGripperPosR;
    // control flag
    ljh::tools::MotorControlTypeSeries<24> JointCntlFlag;
// output end
    // controller 
    double Ts;
    double SimulationTime;
    // BHR::YouYi_ControlFrame::FrictionTest::ControlFrame controller;
    BHR::YouYi_ControlFrame::_youyi_walk_basic::YOUYIControlFrame controller;
public:
// 使用controller ts 控制器及控制器参数来进行构造
    control_interface(double ts);
    void set_input(const vector<double> & input_date);
    double getSimulationTime();
    void crouch_pos();
    void init();
    void run();
    void clear();
    vector<double> get_output();
    ~control_interface();
};

#endif