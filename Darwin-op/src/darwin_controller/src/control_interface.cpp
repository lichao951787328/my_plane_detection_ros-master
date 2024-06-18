#include <darwin_controller/control_interface.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>
control_interface::control_interface(double ts)
{
    Ts = ts;
    // controller = BHR::YouYi_ControlFrame::FrictionTest::ControlFrame(Ts);
    controller = BHR::YouYi_ControlFrame::_youyi_walk_basic::YOUYIControlFrame(Ts);
}

// 需要四元数转欧拉角吗？怎么说明转换顺序
void control_interface::set_input(const vector<double> & input_date)
{
    // 腿 6*2， 六维力传感器 6*2， imu 10 key 1 clock 1
    // LOG(INFO)<<"set conntrol input data";
    CHECK(input_date.size() == 36);
    // left leg
    for (size_t i = 0; i < 6; i++)
    {
        real_joint_left_leg[i] = input_date.at(i);
    }
    // right leg
    for (size_t i = 6; i < 12; i++)
    {
        real_joint_right_leg[i - 6] = input_date.at(i);
    }
    // left force sensor
    for (size_t i = 12; i < 15; i++)
    {
        // LOG(INFO)<<"left force sensor: "<<input_date.at(i);
        FSForceLeftFoot[i-12] = input_date.at(i);
        // JFSAnkleForceL[i-12] = input_date.at(i);
    }
    for (size_t i = 15; i < 18; i++)
    {
        // LOG(INFO)<<"left torque: "<<input_date.at(i);
        FSTorqueLeftFoot[i-15] = input_date.at(i);
        // JFSAnkleTorqueL[i-15] = input_date.at(i);
    }
    // right force sensor
    for (size_t i = 18; i < 21; i++)
    {
        // LOG(INFO)<<"right force sensor: "<<input_date.at(i);
        FSForceRightFoot[i-18] = input_date.at(i);
        // JFSAnkleForceR[i-18] = input_date.at(i);
    }
    for (size_t i = 21; i < 24; i++)
    {
        // LOG(INFO)<<"right torque: "<<input_date.at(i);
        FSTorqueRightFoot[i-21] = input_date.at(i);
        // JFSAnkleTorqueR[i-21] = input_date.at(i);
    }

    // ang
    // LOG(INFO)<<"q: "<<input_date.at(27)<<" "<<input_date.at(28)<<" "<<input_date.at(29)<<" "<<input_date.at(30);
    Eigen::Quaternionf qf(input_date.at(27), input_date.at(28), input_date.at(29), input_date.at(30));
    // qf.w() = input_date.at(27); 
    // LOG(INFO)<<"QF: "<<qf.w()<<" "<<qf.x()<<" "<<qf.y()<<" "<<qf.z();
    Eigen::Vector3f ealur = qf.toRotationMatrix().eulerAngles(2,1,0);
    Ang[0] = ealur.x();
    Ang[1] = ealur.y();
    Ang[2] = ealur.z();
    // LOG(INFO)<<"pose: "<<Ang[0]<<" "<<Ang[1]<<" "<<Ang[2];

    key = (int)input_date.at(34);
    // LOG(INFO)<<"CONTROL USE "<<key;
    SimulationTime = input_date.at(35);
}

double control_interface::getSimulationTime()
{
    return SimulationTime;
}


void control_interface::crouch_pos()
{
    real_joint_left_leg[2] = -10. /180. * 3.1415926535;
    real_joint_left_leg[3] =  20. /180. * 3.1415926535;
    real_joint_left_leg[4] = -10. /180. * 3.1415926535;

    real_joint_right_leg[2] = -10. /180. * 3.1415926535;
    real_joint_right_leg[3] =  20. /180. * 3.1415926535;
    real_joint_right_leg[4] = -10. /180. * 3.1415926535;

    ref_joint_left_leg[2] = -10. /180. * 3.1415926535;
    ref_joint_left_leg[3] =  20. /180. * 3.1415926535;
    ref_joint_left_leg[4] = -10. /180. * 3.1415926535;

    ref_joint_right_leg[2] = -10. /180. * 3.1415926535;
    ref_joint_right_leg[3] =  20. /180. * 3.1415926535;
    ref_joint_right_leg[4] = -10. /180. * 3.1415926535;
}

void control_interface::init()
{
    
    controller.setInput({
        &RealGripperPosL,
        &RealGripperPosR,

        real_joint_left_leg,
        real_joint_right_leg,
        real_joint_left_arm,
        real_joint_right_arm,

        FSForceLeftFoot,
        FSForceRightFoot,
        FSTorqueLeftFoot,
        FSTorqueRightFoot,

        FSForceLeftWrist,
        FSForceRightWrist,
        FSTorqueLeftWrist,
        FSTorqueRightWrist,

        FSForceLeftHip,
        FSForceRightHip,
        FSTorqueLeftHip,
        FSTorqueRightHip,

        JFSKneeForceL,
        JFSKneeForceR,
        JFSKneeTorqueL,
        JFSKneeTorqueR,

        JFSAnkleForceL,
        JFSAnkleForceR,
        JFSAnkleTorqueL,
        JFSAnkleTorqueR,

        Ang,
        Acc,
        Omg,

        &key,
        &Logger
    });

    controller.setOutput({
        ref_joint_left_leg,
        ref_joint_right_leg,
        ref_joint_left_arm,
        ref_joint_right_arm,
        RefTorqueLegJointL,
        RefTorqueLegjointR,
        RefTorqueArmJointL,
        RefTorqueArmJointR,
        &RefGripperPosL,
        &RefGripperPosR,
        &JointCntlFlag});
    controller.init();
    
}

void control_interface::run()
{
    // auto flag = controller.getControlStateFlag();
    // switch (flag)
    // {
    // case ljh::tools::ControlType::PositionControl :
    //     _IO.ControlStateFlag = lee::MOTORControlState::POSITIONCONTROL;
    //     break;
    // case ljh::tools::ControlType::TorqueControl :
    //     _IO.ControlStateFlag = lee::MOTORControlState::TORQUECONTROL;
    //     break;
    // case ljh::tools::ControlType::StandStill :
    //     _IO.ControlStateFlag = lee::MOTORControlState::POSITIONCONTROL;
    // default:
    //     break;
    // }
    // 这是干啥的 发布
    // transJointFromCopp(_IO.RealJoint, RealJoint);
    // LOG(INFO)<<"control sim run";
    // LOG(INFO)<<"left foot force: "<<FSForceLeftFoot[0]<<" "<<FSForceLeftFoot[1]<<" "<<FSForceLeftFoot[2];
    // LOG(INFO)<<"left foot torque: "<<FSTorqueLeftFoot[0]<<" "<<FSTorqueLeftFoot[1]<<" "<<FSTorqueLeftFoot[2];
    // LOG(INFO)<<"right foot force: "<<FSForceRightFoot[0]<<" "<<FSForceRightFoot[1]<<" "<<FSForceRightFoot[2];
    // LOG(INFO)<<"right foot torque: "<<FSTorqueRightFoot[0]<<" "<<FSTorqueRightFoot[1]<<" "<<FSTorqueRightFoot[2];
    controller.run();
    controller.log();

    // test

    // ref_joint_left_leg[2] = -10. /180. * 3.1415926535;
    // ref_joint_left_leg[3] =  20. /180. * 3.1415926535;
    // ref_joint_left_leg[4] = -10. /180. * 3.1415926535;

    // ref_joint_right_leg[2] = -10. /180. * 3.1415926535;
    // ref_joint_right_leg[3] =  20. /180. * 3.1415926535;
    // ref_joint_right_leg[4] = -10. /180. * 3.1415926535;


    // if (SimulationTime >= 1)
    // {
    //     // switch (_IO.ControlStateFlag)
    //     // {
    //     // case lee::MOTORControlState::POSITIONCONTROL:
    //     //     transJointToCopp(RefJoint, _IO.RefJoint);
    //     //     break;
    //     // case lee::MOTORControlState::TORQUECONTROL:
    //     //     transJointToCopp(RefJoint, _IO.RefTorque);
    //     //     break;
    //     // default:
    //     //     break;
    //     // }
    // }
}

void control_interface::clear()
{
    LOG(INFO)<<"start write data";
    controller.clear();
}

vector<double> control_interface::get_output()
{
    vector<double> output;
    for (size_t i = 0; i < 6; i++)
    {
        output.emplace_back(ref_joint_left_leg[i]);
    }
    for (size_t i = 0; i < 6; i++)
    {
        output.emplace_back(ref_joint_right_leg[i]);
    }
    return output;
}


control_interface::~control_interface()
{
}
