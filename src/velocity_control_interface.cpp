#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "labview_interface_programs/velocitycommand.h"
#include <iostream>

const double pi=3.141592654;


ros::Publisher pub;

void callback(const geometry_msgs::Twist::ConstPtr &ros_vel){
labview_interface_programs::velocitycommand lab_vel;

lab_vel.angular=(ros_vel->angular.z)*(180.0/pi);

lab_vel.linear=(ros_vel->linear.x)*1000.0;

pub.publish(lab_vel);
}





int main(int argc, char**argv){

ros::init(argc,argv,"velocity_control_interface_node");

ros::NodeHandle n;

ros::Subscriber sub=n.subscribe("cmd_vel",100,callback);

pub=n.advertise<labview_interface_programs::velocitycommand>("lab_command_velocities",100);

ros::spin();

return 0;
}
