#ifndef CONTROL_H_
#define CONTROL_H_
#include <ros/ros.h>
#include <string>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>
#include <ros/callback_queue.h>
#include <rosgraph_msgs/Clock.h>
#include <vector>
#include <thread>
#include <mutex>
#include <Eigen/Core>
#include <keyboard/Key.h>
using namespace std;

struct inter_date
{
    float left_leg[6] = {0.};
    float right_leg[6] = {0.};
    float ft_l[6] = {0.};
    float ft_r[6] = {0.};
    float imu_ang[4] = {0.};// 姿态
    float imu_acc[3] = {0.};
    float imu_omg[3] = {0.};
    int Key = 0;
    float clock_relative;
};

class control_inter
{
private:
    // 数据交互的topic
    string joint_state_topic;
    vector<string> ft_sensor_topics;
    string imu_topic;
    string clock_topic;
    string keydown_topic;
    string keyup_topic;
    rosgraph_msgs::Clock start_time;
    bool is_start = true;

    // 节点句柄
    ros::NodeHandle joint_state_nh;
    ros::NodeHandle ft_l_nh;
    ros::NodeHandle ft_r_nh;
    ros::NodeHandle imu_nh;
    ros::NodeHandle clock_nh;
    ros::NodeHandle keydown_nh;
    ros::NodeHandle keyup_nh;

    // 回调队列
    ros::CallbackQueue joint_state_cq;
    ros::CallbackQueue ft_l_cq;
    ros::CallbackQueue ft_r_cq;
    ros::CallbackQueue imu_cq;
    ros::CallbackQueue clock_cq;
    ros::CallbackQueue keydown_cq;
    ros::CallbackQueue keyup_cq;

    
    // 订阅器
    ros::Subscriber joint_state_sub;
    ros::Subscriber ft_l_sub;
    ros::Subscriber ft_r_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber clock_sub;
    ros::Subscriber keydown_sub;
    ros::Subscriber keyup_sub;

    // 回调线程
    std::thread spinner_thread_jointstate;
    std::thread spinner_thread_ft_l;
    std::thread spinner_thread_ft_r;
    std::thread spinner_thread_imu;
    std::thread spinner_thread_keydown;
    std::thread spinner_thread_keyup;
    std::thread spinner_thread_clock;

    // 回调spinner
    ros::SingleThreadedSpinner joint_state_spinner;
    ros::SingleThreadedSpinner ft_l_spinner;
    ros::SingleThreadedSpinner ft_r_spinner;
    ros::SingleThreadedSpinner imu_spinner;
    ros::SingleThreadedSpinner clock_spinner;
    ros::SingleThreadedSpinner keydown_spinner;
    ros::SingleThreadedSpinner keyup_spinner;
    
    // 存储的是YouYi机器人坐标系下的值
    inter_date date;
    std::mutex m;
    // 表示由达尔文机器人下的踝坐标系到YouYi机器人坐标系下的转换矩阵
    Eigen::Matrix3f left_Rot, right_Rot;
public:
    control_inter(ros::NodeHandle js_nh, ros::NodeHandle fl_nh, ros::NodeHandle fr_nh, ros::NodeHandle im_nh, ros::NodeHandle cl_nh, ros::NodeHandle kd_nh, ros::NodeHandle ku_nh);

    void setTopics(string & joint_state_topic_, vector<string> & ft_sensor_topics_, string & imu_topic_, string & clock_topic_, string & keydown_topic_, string & keyup_topic_);
    
    void initial();

    void cb_joint_state(sensor_msgs::JointStateConstPtr jointstaeP);
    void callback_ft_l(geometry_msgs::WrenchStampedConstPtr ftP);
    void callback_ft_r(geometry_msgs::WrenchStampedConstPtr ftP);
    void callback_imu(sensor_msgs::ImuConstPtr imuP);
    void callback_clock(rosgraph_msgs::ClockPtr clockP);
    void callback_keydown(keyboard::KeyConstPtr keydownP);
    void callback_keyup(keyboard::KeyConstPtr keyupP);

    void joint_state_spin();
    void ft_l_spin();
    void ft_r_spin();
    void imu_spin();
    void clock_spin();
    void keydown_spin();
    void keyup_spin();
    // void get_keydown();
    // void get_keyup();

    void pub_spin();

    void refresh_ref_date();
    void print_data();

    // void set_target_date(vector<float> left_leg, vector<float> right_leg);
    // void publish_date();
    vector<double> get_date();

    ~control_inter();
};




#endif