#include <ros/ros.h>
#include <string>
#include <iostream>
#include <std_msgs/Float64.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>
#include <thread>
using namespace std;

void callback_jointstate(sensor_msgs::JointStateConstPtr jointstaeP)
{
    
}

void callback_ft_l(geometry_msgs::WrenchStampedConstPtr ftP)
{

}

void callback_ft_r(geometry_msgs::WrenchStampedConstPtr ftP)
{
    
}

void callback_imu(sensor_msgs::ImuConstPtr imuP)
{
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "darwin_controller");
    ros::NodeHandle node_handle;
    // 控制关节运动的发布器             
    string command_topics[] = 
    {
        "/darwin/j_pan_position_controller/command",
        "/darwin/j_tilt_position_controller/command",
        "/darwin/j_gripper_l_position_controller/command",
        "/darwin/j_gripper_r_position_controller/command",
        // left leg 6
        "/darwin/j_pelvis_l_position_controller/command",
        "/darwin/j_thigh1_l_position_controller/command",
        "/darwin/j_thigh2_l_position_controller/command",
        "/darwin/j_tibia_l_position_controller/command",
        "/darwin/j_ankle1_l_position_controller/command",
        "/darwin/j_ankle2_l_position_controller/command",
        // right leg 6
        "/darwin/j_pelvis_r_position_controller/command",
        "/darwin/j_thigh1_r_position_controller/command",
        "/darwin/j_thigh2_r_position_controller/command",
        "/darwin/j_tibia_r_position_controller/command",
        "/darwin/j_ankle1_r_position_controller/command",
        "/darwin/j_ankle2_r_position_controller/command",
        // left arm 4
        "/darwin/j_shoulder_l_position_controller/command",
        "/darwin/j_high_arm_l_position_controller/command",
        "/darwin/j_low_arm_l_position_controller/command",
        "/darwin/j_wrist_l_position_controller/command",
        // right arm 4
        "/darwin/j_shoulder_r_position_controller/command",
        "/darwin/j_high_arm_r_position_controller/command",
        "/darwin/j_low_arm_r_position_controller/command",
        "/darwin/j_wrist_r_position_controller/command"
    };

    // 收集关节实际位置的订阅器 话题：/darwin/joint_states， 消息类型：sensor_msgs/JointState
    // - j_ankle1_l - j_ankle1_r - j_ankle2_l - j_ankle2_r - j_gripper_l - j_gripper_r - j_high_arm_l - j_high_arm_r - j_low_arm_l - j_low_arm_r - j_pan - j_pelvis_l - j_pelvis_r - j_shoulder_l - j_shoulder_r - j_thigh1_l - j_thigh1_r - j_thigh2_l - j_thigh2_r - j_tibia_l - j_tibia_r - j_tilt - j_wrist_l - j_wrist_r
    string joint_state_sub_topic = "/darwin/joint_states";
    // 收集力力矩传感器的订阅器 话题：/ft_sensor_l，消息类型：geometry_msgs/WrenchStamped 话题：/ft_sensor_r，消息类型：geometry_msgs/WrenchStamped
    string ft_sensor_sub_topic[2] = {"/ft_sensor_l", "/ft_sensor_r"};
    // 收集IMU数据的订阅器 /imu sensor_msgs/Imu
    string imu_sub_topic = "/imu";
    vector<ros::Publisher> publishers;
    for (size_t i = 0; i < 24; i++)
    {
        ros::Publisher new_publisher = node_handle.advertise<std_msgs::Float64>(command_topics[i],1);
        publishers.emplace_back(new_publisher);
    }

    ros::NodeHandle nh_jointstate;
    ros::CallbackQueue callback_queue_jointstate;
    nh_jointstate.setCallbackQueue(&callback_queue_jointstate);
    ros::Subscriber joint_state_sub = nh_jointstate.subscribe(joint_state_sub_topic, 1, callback_jointstate);
    std::thread spinner_thread_jointstate([&callback_queue_jointstate]() {
    ros::SingleThreadedSpinner spinner_jointstate;
    spinner_jointstate.spin(&callback_queue_jointstate);
    });

    ros::NodeHandle nh_ft_l;
    ros::CallbackQueue callback_queue_ft_l;
    nh_ft_l.setCallbackQueue(&callback_queue_ft_l);
    ros::Subscriber ft_l_sub = nh_ft_l.subscribe(ft_sensor_sub_topic[0], 1, callback_ft_l);
    std::thread spinner_thread_ft_l([&callback_queue_ft_l]() {
    ros::SingleThreadedSpinner spinner_ft_l;    
    spinner_ft_l.spin(&callback_queue_ft_l);
    });

    ros::NodeHandle nh_ft_r;
    ros::CallbackQueue callback_queue_ft_r;
    nh_ft_r.setCallbackQueue(&callback_queue_ft_r);
    ros::Subscriber ft_r_sub = nh_ft_r.subscribe(ft_sensor_sub_topic[1], 1, callback_ft_r);
    std::thread spinner_thread_ft_r([&callback_queue_ft_r]() {
    ros::SingleThreadedSpinner spinner_ft_r;
    spinner_ft_r.spin(&callback_queue_ft_r);
    });

    ros::NodeHandle nh_imu;
    ros::CallbackQueue callback_queue_imu;
    nh_imu.setCallbackQueue(&callback_queue_imu);
    ros::Subscriber imu_sub = nh_imu.subscribe(imu_sub_topic, 1, callback_imu);
    std::thread spinner_thread_imu([&callback_queue_imu]() {
        ros::SingleThreadedSpinner spinner_imu;
        spinner_imu.spin(&callback_queue_imu);
        });
    spinner_thread_jointstate.join();
    spinner_thread_ft_l.join();
    spinner_thread_ft_r.join();
    spinner_thread_imu.join();

    ros::Rate loop_rate(250);
    int i = 1;
    while (ros::ok())
    {
        // compute joints angles

        // publish joint angle commands
        for (size_t i = 17; i < 24; i++)
        {
            // if(!isnan())
            // {
            //     std_msgs::Float64 command_message;
            //     command_message.data = ;
            //     publishers.at(i).publish(command_message);
            // }
            std_msgs::Float64 command_message;
            command_message.data = 0.01 * i;
            publishers.at(i).publish(command_message);
        }
        
        ros::spinOnce();
        loop_rate.sleep();
        i++;
        if (i==3000)
        {
            break;
        }
        
    }
    

    return 0;
}