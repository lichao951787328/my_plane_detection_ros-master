#include <control.h>
#include <glog/logging.h>
#include <std_msgs/Float64.h>
#include <iostream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "inter_face");
    ros::NodeHandle main_nh;
    ros::NodeHandle jointstate_nh;
    ros::NodeHandle forcesensor_left_nh;
    ros::NodeHandle forcesensor_right_nh;
    ros::NodeHandle imu_nh;
    ros::NodeHandle clock_nh;
    ros::NodeHandle keydown_nh;
    ros::NodeHandle keyup_nh;
    // 交互控制
    control_inter controller(jointstate_nh, forcesensor_left_nh, forcesensor_right_nh, imu_nh, clock_nh, keydown_nh, keyup_nh);

    vector<string> left_leg_topics;
    vector<string> right_leg_topics;
    string joint_state_topic;
    vector<string> ft_sensor_topics;
    string imu_topic, clock_topic, keydown_topic, keyup_topic;
    left_leg_topics.emplace_back("/darwin/j_pelvis_l_position_controller/command");
    left_leg_topics.emplace_back("/darwin/j_thigh1_l_position_controller/command");
    left_leg_topics.emplace_back("/darwin/j_thigh2_l_position_controller/command");
    left_leg_topics.emplace_back("/darwin/j_tibia_l_position_controller/command");
    left_leg_topics.emplace_back("/darwin/j_ankle1_l_position_controller/command");
    left_leg_topics.emplace_back("/darwin/j_ankle2_l_position_controller/command");

    right_leg_topics.emplace_back("/darwin/j_pelvis_r_position_controller/command");
    right_leg_topics.emplace_back("/darwin/j_thigh1_r_position_controller/command");
    right_leg_topics.emplace_back("/darwin/j_thigh2_r_position_controller/command");
    right_leg_topics.emplace_back("/darwin/j_tibia_r_position_controller/command");
    right_leg_topics.emplace_back("/darwin/j_ankle1_r_position_controller/command");
    right_leg_topics.emplace_back("/darwin/j_ankle2_r_position_controller/command");

    joint_state_topic = "/darwin/joint_states";
    ft_sensor_topics.emplace_back("/ft_sensor_l");
    ft_sensor_topics.emplace_back("/ft_sensor_r");
    imu_topic = "imu";
    clock_topic = "/clock";
    keydown_topic = "/keyboard/keydown";
    keyup_topic = "/keyboard/keyup";
    controller.setTopics(joint_state_topic, ft_sensor_topics, imu_topic, clock_topic, keydown_topic, keyup_topic);
    controller.initial();

    vector<ros::Publisher> left_leg_publishers(6);
    vector<ros::Publisher> right_leg_publishers(6);
    for (size_t i = 0; i < 6; i++)
    {
        left_leg_publishers.at(i) = main_nh.advertise<std_msgs::Float64>(left_leg_topics.at(i),1);
        right_leg_publishers.at(i) = main_nh.advertise<std_msgs::Float64>(right_leg_topics.at(i),1);
    }
    ros::Rate loop_rate(200);
    while (ros::ok())
    {
        LOG(INFO)<<"print data";
        // controller.print_data();
        ros::spinOnce();
        loop_rate.sleep();
        // usleep()
    }
   

    controller.refresh_ref_date();
    return 0;
}