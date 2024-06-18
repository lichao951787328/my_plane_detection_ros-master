#include <ros/ros.h>
#include <darwin_controller/control.h>
#include <darwin_controller/control_interface.h>
#include <glog/logging.h>
#include <std_msgs/Float64.h>
#include <fstream>
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
    // vector<string> left_arm_topics;
    // vector<string> right_arm_topics;
    // vector<string> other_joints_topics;
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

    // left_arm_topics.emplace_back("/darwin/j_shoulder_l_position_controller/command");
    // left_arm_topics.emplace_back("/darwin/j_high_arm_l_position_controller/command");
    // left_arm_topics.emplace_back("/darwin/j_low_arm_l_position_controller/command");
    // left_arm_topics.emplace_back("/darwin/j_wrist_l_position_controller/command");
    // right_arm_topics.emplace_back("/darwin/j_shoulder_r_position_controller/command");
    // right_arm_topics.emplace_back("/darwin/j_high_arm_r_position_controller/command");
    // right_arm_topics.emplace_back("/darwin/j_low_arm_r_position_controller/command");
    // right_arm_topics.emplace_back("/darwin/j_wrist_r_position_controller/command");
    // other_joints_topics.emplace_back("/darwin/j_pan_position_controller/command");
    // other_joints_topics.emplace_back("/darwin/j_tilt_position_controller/command");
    // other_joints_topics.emplace_back("/darwin/j_gripper_l_position_controller/command");
    // other_joints_topics.emplace_back("/darwin/j_gripper_r_position_controller/command");

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
    ros::Rate loop_rate = 1000;
    control_interface control_sim(0.001);//底层控制
    
    // 打开文件并保存
    std::ofstream fw;
    fw.open("/home/lichao/Darwin-op/src/darwin_controller/publish_joint.txt");
    LOG(INFO)<<"init control";
    control_sim.crouch_pos();
    // 计算关节目标位置
    bool is_init = false;
    vector<float> left(6, 0.0);
    vector<float> right(6, 0.0);
    while (ros::ok())
    {
        ros::spinOnce();
        if (control_sim.getSimulationTime() < 2 && !is_init)
        {
            control_sim.init();
            vector<double> ref_legs = control_sim.get_output();
            CHECK(ref_legs.size() == 12);
            // 将YouYi坐标系下的关节角度值转到达尔文机器人坐标系下
            left.at(0)  = - ref_legs.at(0);
            left.at(1)  = - ref_legs.at(1);
            left.at(2)  = - ref_legs.at(2);
            left.at(3)  = - ref_legs.at(3);
            left.at(4)  =   ref_legs.at(4);
            left.at(5)  =   ref_legs.at(5);
            right.at(0) = - ref_legs.at(6);
            right.at(1) = - ref_legs.at(7);
            right.at(2) =   ref_legs.at(8);
            right.at(3) =   ref_legs.at(9);
            right.at(4) = - ref_legs.at(10);
            right.at(5) =   ref_legs.at(11);
            is_init = true;
            // return 0;
        }
        else
        {
            vector<double> input_data = controller.get_date();
            if (input_data.at(34) == 32)// 空格退出
            {
                LOG(INFO)<<"SPACE:..........................";
                control_sim.clear();
                return 0;
            }
            else
            {
                control_sim.set_input(input_data);
                LOG(INFO)<<"loop control";
                control_sim.run();
                vector<double> ref_legs = control_sim.get_output();
                // vector<float> left(6, 0.0);
                // vector<float> right(6, 0.0);
                CHECK(ref_legs.size() == 12);
                left.at(0)  = - ref_legs.at(0);
                left.at(1)  = - ref_legs.at(1);
                left.at(2)  = - ref_legs.at(2);
                left.at(3)  = - ref_legs.at(3);
                left.at(4)  =   ref_legs.at(4);
                left.at(5)  =   ref_legs.at(5);
                right.at(0) = - ref_legs.at(6);
                right.at(1) = - ref_legs.at(7);
                right.at(2) =   ref_legs.at(8);
                right.at(3) =   ref_legs.at(9);
                right.at(4) = - ref_legs.at(10);
                right.at(5) =   ref_legs.at(11);
            }
        }
        LOG(INFO)<<"enter publisher";
        for (size_t i = 0; i < 6; i++)
        {
            std_msgs::Float64 joint_l, joint_r;
            joint_l.data = left.at(i);
            joint_r.data = right.at(i);
            left_leg_publishers.at(i).publish(joint_l);
            right_leg_publishers.at(i).publish(joint_r);
        }
        for (size_t i = 0; i < 6; i++)
        {
            fw<<left.at(i)<<" ";
        }

        for (size_t i = 0; i < 6; i++)
        {
            fw<<right.at(i)<<" ";
        }
        fw<<endl;
        loop_rate.sleep();
    }
    controller.refresh_ref_date();
    return 0;
}