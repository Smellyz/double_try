#include <ros/ros.h>
#include "robot_msgs/ShootInfo.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "shoot_info_test");
    ros::NodeHandle n;
    ros::Publisher robot_0 = n.advertise<robot_msgs::ShootInfo>("shoot_info_0",1000);
    ros::Publisher robot_2 = n.advertise<robot_msgs::ShootInfo>("shoot_info_2",1000);
    ros::Publisher robot_4 = n.advertise<robot_msgs::ShootInfo>("shoot_info_4",1000);
    ros::Publisher robot_6 = n.advertise<robot_msgs::ShootInfo>("shoot_info_6",1000);

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        robot_msgs::ShootInfo shoot_test;
        shoot_test.run_friction_wheel = true;
        shoot_test.shoot_state =  1;
        shoot_test.friction_wheel_speed = 20;
        shoot_test.shoot_freq = 1;
        shoot_test.sent_bullet = 100;

        robot_0.publish(shoot_test); 
        robot_2.publish(shoot_test); 
        robot_4.publish(shoot_test); 
        robot_6.publish(shoot_test);        

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
    
}