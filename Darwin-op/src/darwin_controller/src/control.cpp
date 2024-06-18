#include <darwin_controller/control.h>
#include <glog/logging.h>
#include <std_msgs/Float64.h>
// #include <conio.h>
#include "conio_lc.h"
#include <unistd.h>
control_inter::control_inter(ros::NodeHandle js_nh, ros::NodeHandle fl_nh, ros::NodeHandle fr_nh, ros::NodeHandle im_nh, ros::NodeHandle cl_nh, ros::NodeHandle kd_nh, ros::NodeHandle ku_nh):joint_state_nh(js_nh),ft_l_nh(fl_nh),ft_r_nh(fr_nh),imu_nh(im_nh),clock_nh(cl_nh),keydown_nh(kd_nh),keyup_nh(ku_nh)
{
}

void control_inter::setTopics(string & joint_state_topic_, vector<string> & ft_sensor_topics_, string & imu_topic_, string & clock_topic_, string & keydown_topic_, string & keyup_topic_)
{
    joint_state_topic = joint_state_topic_;
    CHECK(ft_sensor_topics_.size() == 2);
    ft_sensor_topics = ft_sensor_topics_;
    imu_topic = imu_topic_;
    clock_topic = clock_topic_;
    keydown_topic = keydown_topic_;
    keyup_topic = keyup_topic_;
}

void control_inter::initial()
{
    LOG(INFO)<<"enter initial";
    left_Rot<<0,0,1,
              0,-1,0,
              1,0,0;
    right_Rot<<0,0,1,
                0,1,0,
                -1,0,0;
    joint_state_nh.setCallbackQueue(&(control_inter::joint_state_cq));
    joint_state_sub = joint_state_nh.subscribe(joint_state_topic, 1, &control_inter::cb_joint_state, this);
    ft_l_nh.setCallbackQueue(&(control_inter::ft_l_cq));
    ft_l_sub = ft_l_nh.subscribe(ft_sensor_topics.at(0), 1, &control_inter::callback_ft_l, this);
    ft_r_nh.setCallbackQueue(&(control_inter::ft_r_cq));
    ft_r_sub = ft_r_nh.subscribe(ft_sensor_topics.at(1), 1, &control_inter::callback_ft_r, this);
    imu_nh.setCallbackQueue(&(control_inter::imu_cq));
    imu_sub = imu_nh.subscribe(imu_topic, 1, &control_inter::callback_imu, this);
    clock_nh.setCallbackQueue(&(control_inter::clock_cq));
    clock_sub = clock_nh.subscribe(clock_topic, 1, &control_inter::callback_clock, this);
    keydown_nh.setCallbackQueue(&(control_inter::keydown_cq));
    keydown_sub = keydown_nh.subscribe(keydown_topic, 1, &control_inter::callback_keydown, this);
    keyup_nh.setCallbackQueue(&(control_inter::keyup_cq));
    keyup_sub = keyup_nh.subscribe(keyup_topic, 1, &control_inter::callback_keyup, this);

    spinner_thread_jointstate = std::thread(&control_inter::joint_state_spin,this);
    spinner_thread_ft_l = std::thread(&control_inter::ft_l_spin, this);
    spinner_thread_ft_r = std::thread(&control_inter::ft_r_spin, this);
    spinner_thread_imu = std::thread(&control_inter::imu_spin, this);
    spinner_thread_clock = std::thread(&control_inter::clock_spin, this);
    spinner_thread_keydown = std::thread(&control_inter::keydown_spin, this);
    spinner_thread_keyup = std::thread(&control_inter::keyup_spin, this);
    // thread_key = std::thread(&control_inter::get_key, this);
    usleep(10000);
}

// - j_ankle1_l - j_ankle1_r - j_ankle2_l - j_ankle2_r - j_gripper_l - j_gripper_r - j_high_arm_l -7 j_high_arm_r - j_low_arm_l - j_low_arm_r - j_pan - j_pelvis_l - j_pelvis_r - j_shoulder_l - j_shoulder_r -8 j_thigh1_l - j_thigh1_r - j_thigh2_l - j_thigh2_r - j_tibia_l - j_tibia_r - j_tilt - j_wrist_l - j_wrist_r9
void control_inter::cb_joint_state(sensor_msgs::JointStateConstPtr jointstaeP)
{
    // LOG(INFO)<<"enter joint state call back.";
    lock_guard<mutex> lockGuard(m);
    // 负号是由于机器人坐标系与YouYi机器人坐标系不同导致的
    date.left_leg[0] = - jointstaeP->position.at(11);//左一 j_pelvis_l
    date.left_leg[1] = - jointstaeP->position.at(15);//左二 j_thigh1_l
    date.left_leg[2] = - jointstaeP->position.at(17);//左三 j_thigh2_l
    date.left_leg[3] = - jointstaeP->position.at(19);//左四 j_tibia_l
    date.left_leg[4] =   jointstaeP->position.at(0); //左五 j_ankle1_l
    date.left_leg[5] =   jointstaeP->position.at(2); //左六 j_ankle2_l

    date.right_leg[0] = - jointstaeP->position.at(12);//右一 j_pelvis_r
    date.right_leg[1] = - jointstaeP->position.at(16);//右二 j_thigh1_r
    date.right_leg[2] =   jointstaeP->position.at(18);//右三 j_thigh2_r
    date.right_leg[3] =   jointstaeP->position.at(20);//右四 j_tibia_r
    date.right_leg[4] = - jointstaeP->position.at(1); //右五 j_ankle1_r
    date.right_leg[5] =   jointstaeP->position.at(3); //右六 j_ankle2_r
}

void control_inter::callback_ft_l(geometry_msgs::WrenchStampedConstPtr ftP)
{
    // LOG(INFO)<<"enter force sensor left call back";
    Eigen::Vector3f force(ftP->wrench.force.x, ftP->wrench.force.y, ftP->wrench.force.z);
    Eigen::Vector3f torque(ftP->wrench.torque.x, ftP->wrench.torque.y, ftP->wrench.torque.z);
    Eigen::Vector3f force_YY = left_Rot*force;
    Eigen::Vector3f torque_YY = left_Rot*torque;
    lock_guard<mutex> lockGuard(m);
    date.ft_l[0] = force_YY.x();
    date.ft_l[1] = force_YY.y();
    date.ft_l[2] = force_YY.z();
    date.ft_l[3] = torque_YY.x();
    date.ft_l[4] = torque_YY.y();
    date.ft_l[5] = torque_YY.z();
}

void control_inter::callback_ft_r(geometry_msgs::WrenchStampedConstPtr ftP)
{
    // LOG(INFO)<<"enter force sensor right call back";
    Eigen::Vector3f force(ftP->wrench.force.x, ftP->wrench.force.y, ftP->wrench.force.z);
    Eigen::Vector3f torque(ftP->wrench.torque.x, ftP->wrench.torque.y, ftP->wrench.torque.z);
    Eigen::Vector3f force_YY = right_Rot*force;
    Eigen::Vector3f torque_YY = right_Rot*torque;
    lock_guard<mutex> lockGuard(m);
    // 左脚踝坐标系 x向上，y向右，z向前
    date.ft_r[0] = force_YY.x();
    date.ft_r[1] = force_YY.y();
    date.ft_r[2] = force_YY.z();
    date.ft_r[3] = torque_YY.x();
    date.ft_r[4] = torque_YY.y();
    date.ft_r[5] = torque_YY.z();
}

void control_inter::callback_imu(sensor_msgs::ImuConstPtr imuP)
{
    // LOG(INFO)<<"enter imu call back";
    lock_guard<mutex> lockGuard(m);
    // 右脚踝坐标系 x向下，y向左，z向前
    date.imu_acc[0] = imuP->linear_acceleration.x;
    date.imu_acc[1] = imuP->linear_acceleration.x;
    date.imu_acc[2] = imuP->linear_acceleration.x;
    date.imu_ang[0] = imuP->orientation.w;
    date.imu_ang[1] = imuP->orientation.x;
    date.imu_ang[2] = imuP->orientation.y;
    date.imu_ang[3] = imuP->orientation.z;
    date.imu_omg[0] = imuP->angular_velocity.x;
    date.imu_omg[1] = imuP->angular_velocity.y;
    date.imu_omg[2] = imuP->angular_velocity.z;
}

void control_inter::callback_clock(rosgraph_msgs::ClockPtr clockP)
{
    // LOG(INFO)<<"enter clock call back";
    lock_guard<mutex> lockGuard(m);
    if (is_start)
    {
        date.clock_relative = 0;
        start_time = *clockP;
        is_start = false;
    }
    else
    {
        date.clock_relative = (*clockP).clock.toSec() - start_time.clock.toSec();
        // LOG(INFO)<<"date.clock_relative: "<<date.clock_relative;
    }
}

void control_inter::callback_keydown(keyboard::KeyConstPtr keydownP)
{
    LOG(INFO)<<"enter key call back";
    lock_guard<mutex> lockGuard(m);
    date.Key = (keydownP->code);
    LOG(INFO)<<(keydownP->code)<<" "<<date.Key;
}

void control_inter::callback_keyup(keyboard::KeyConstPtr keyupP)
{
    LOG(INFO)<<"enter key call back";
    lock_guard<mutex> lockGuard(m);
    date.Key = (keyupP->code);
    LOG(INFO)<<(keyupP->code)<<" "<<date.Key;
}


void control_inter::joint_state_spin()
{
    joint_state_spinner.spin(&(control_inter::joint_state_cq));
}

void control_inter::ft_l_spin()
{
    ft_l_spinner.spin(&(control_inter::ft_l_cq));
}
    
void control_inter::ft_r_spin()
{
    ft_r_spinner.spin(&(control_inter::ft_r_cq));
}

void control_inter::imu_spin()
{
    imu_spinner.spin(&(control_inter::imu_cq));
}

void control_inter::clock_spin()
{
    clock_spinner.spin(&(control_inter::clock_cq));
}

void control_inter::keydown_spin()
{
    keydown_spinner.spin(&(control_inter::keydown_cq));
}

void control_inter::keyup_spin()
{
    keyup_spinner.spin(&(control_inter::keyup_cq));
}

// void control_inter::get_key()
// {
//     // LOG(INFO)<<"get keyboard";
//     int ch;
//     while (1)
//     {
//         if (kbhit())
//         {
//             ch = getch();
//             if (ch == 27)
//             {
//                 break;
//             }
//         }
//         else
//         {
//             ch = 0;
//         }
//         usleep(5000);
//         // LOG(INFO)<<"key is "<<ch;
//         lock_guard<mutex> lockGuard(m);
//         date.Key = ch;
//         // date.Key = 49;
//     }
// }

void control_inter::refresh_ref_date()
{
    spinner_thread_jointstate.join();
    spinner_thread_ft_l.join();
    spinner_thread_ft_r.join();
    spinner_thread_imu.join();
    spinner_thread_keydown.join();
    spinner_thread_keyup.join();
    spinner_thread_clock.join();
}

vector<double> control_inter::get_date()
{
    // LOG(INFO)<<"get date from gazebo";
    lock_guard<mutex> lockGuard(m);
    vector<double> return_date;
    // left leg
    for (size_t i = 0; i < 6; i++)
    {
        return_date.emplace_back(date.left_leg[i]);
    }
    // right leg
    for (size_t i = 0; i < 6; i++)
    {
        return_date.emplace_back(date.right_leg[i]);
    }
    // left force 
    for (size_t i = 0; i < 6; i++)
    {
        // LOG(INFO)<<"left force: "<<i<<" "<<date.ft_l[i];
        return_date.emplace_back(date.ft_l[i]);
    }
    // right force
    for (size_t i = 0; i < 6; i++)
    {
        // LOG(INFO)<<"right force: "<<i<<" "<<date.ft_r[i];
        return_date.emplace_back(date.ft_r[i]);
    }
    // imu acc
    for (size_t i = 0; i < 3; i++)
    {
        return_date.emplace_back(date.imu_acc[i]);
    }
    // imu ang
    for (size_t i = 0; i < 4; i++)
    {
        return_date.emplace_back(date.imu_ang[i]);
    }
    // imu omg
    for (size_t i = 0; i < 3; i++)
    {
        return_date.emplace_back(date.imu_omg[i]);
    }
    return_date.emplace_back(date.Key);
    // LOG(INFO)<<"INPUT DATA: "<<date.Key;
    return_date.emplace_back(date.clock_relative);
    return return_date;
}

void control_inter::print_data()
{
    cout<<"left leg:"<<endl;
    for (size_t i = 0; i < 6; i++)
    {
        cout<<date.left_leg[i]<<" ";
    }
    cout<<endl;
    cout<<"right leg:"<<endl;
    for (size_t i = 0; i < 6; i++)
    {
        cout<<date.right_leg[i]<<" ";
    }
    cout<<endl;
    cout<<"left force sensor:"<<endl;
    for (size_t i = 0; i < 6; i++)
    {
        cout<<date.ft_l[i]<<" ";
    }
    cout<<endl;
    cout<<"right force sensor:"<<endl;
    for (size_t i = 0; i < 6; i++)
    {
        cout<<date.ft_r[i]<<" ";
    }
    cout<<endl;
    cout<<"imu angle:"<<endl;
    for (size_t i = 0; i < 4; i++)
    {
        cout<<date.imu_ang[i]<<" ";
    }
    cout<<endl;
    cout<<"imu acc:"<<endl;
    for (size_t i = 0; i < 3; i++)
    {
        cout<<date.imu_acc[i]<<" ";
    }
    cout<<endl;
    cout<<"imu omg:"<<endl;
    for (size_t i = 0; i < 3; i++)
    {
        cout<<date.imu_omg[i]<<" ";
    }
    cout<<endl;
    cout<<"key: "<<date.Key<<endl;
    cout<<"time eclips: "<<date.clock_relative<<endl;
}

control_inter::~control_inter()
{
    ft_sensor_topics.clear();
}