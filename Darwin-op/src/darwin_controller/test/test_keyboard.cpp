#include <ros/ros.h>
#include <keyboard/Key.h>
#include <iostream>
using namespace std;
int key = 0;
void getkey(const keyboard::Key & msg )
{
    std::cout<<"msg: "<<msg.code<<endl;
    key = msg.code;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "key");
    ros::NodeHandle n; //创建句柄节点
    ros::Subscriber sub = n.subscribe("/keyboard/keydown", 1000, getkey);
    ros::Rate loop(200);
    while (ros::ok())
    {
        ros::spinOnce();
        cout<<"key: "<<key<<endl;
        loop.sleep();
    }
    
    // ros::spin();
    return 0;
}