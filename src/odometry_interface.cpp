#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include "labview_interface_programs/odometrydata.h"

//Initialise variables here.
double x=0; //in (m)
double y=0;//in (m)
double theta=0; //in (rad)
double linear=0;//in (m/s)
double angular=0;//in (rad/s)

const double pi=3.141592654;

//When the odometry data arrives from labview, update variables.
void callback(const labview_interface_programs::odometrydata::ConstPtr &lab_odom){
//get the current x position relative to the initial position.
x=(lab_odom->rosx)/1000.0;
//get the current y position relative to the initial position.
y=(lab_odom->rosy)/1000.0;
//get the current theta position relative to the intitial heading.
theta=(lab_odom->rostheta);
//get the current linear velocity (in m/s)
linear=(lab_odom->linearvelocity)/1000.0;
//get the current angular velocity (in rad/s)
angular=(lab_odom->angularvelocity)*(pi/180.0);
}


int main(int argc, char **argv){
//initialise the node.
ros::init(argc,argv,"Odometry_interface");
//create the ros nodehandle.
ros::NodeHandle n;
//subscribe to the raw odometry data topic
ros::Subscriber sub=n.subscribe("raw_odometry_data",20,callback);
//advertise the odometry.
ros::Publisher odom_pub=n.advertise<nav_msgs::Odometry>("odom",50);
//create a transform broadcaster
tf::TransformBroadcaster odom_broadcaster;
//run the while loop at a frequency of 100hz
ros::Rate r(100.0);


while(n.ok()){
//get the current ros time.
ros::Time current_time=ros::Time::now();
//refesh the thread
ros::spinOnce();
//convert theta to quaternion.
geometry_msgs::Quaternion odom_quat=tf::createQuaternionMsgFromYaw(theta);
//create a transform stamped variable
geometry_msgs::TransformStamped odom_trans;

//populate the transform variable with the positional values
odom_trans.header.stamp=current_time;
odom_trans.header.frame_id="odom";
odom_trans.child_frame_id="base_link";

odom_trans.transform.translation.x=x;
odom_trans.transform.translation.y=y;
odom_trans.transform.translation.z=0.0;
odom_trans.transform.rotation=odom_quat;

//send the transform
odom_broadcaster.sendTransform(odom_trans);

//we also need to publish positional values and velocity values on the odom topic.
//create a odometry variable
nav_msgs::Odometry odom;

//populate the odometry variable with data
odom.header.stamp=current_time;
odom.header.frame_id="odom";

odom.pose.pose.position.x=x;
odom.pose.pose.position.y=y;
odom.pose.pose.position.z=0.0;
odom.pose.pose.orientation=odom_quat;

odom.child_frame_id="base_link";
odom.twist.twist.linear.x=linear;
odom.twist.twist.linear.y=0.0;
odom.twist.twist.angular.z=angular;

//publish the odometry variable
odom_pub.publish(odom);
// the sleep function delays the loop to ensure that it runs at 100hz. 
r.sleep();


}

return 0;
}
