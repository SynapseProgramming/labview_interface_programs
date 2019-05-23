#include <ros/ros.h>
#include "labview_interface_programs/RawLaser.h"
#include <std_msgs/Bool.h>
#include <iostream>

ros::Publisher pub;



//door threshold is in metres.
double door_threshold;



void lasercb(const labview_interface_programs::RawLaser::ConstPtr &msg){
double sum=0;
double avg=0;

//calculate the average value of data points between +-15 degrees of lidar heading.
for(unsigned int i=238; i<=298;i++){
//divided by 1000 to convert from  mm to metres.
sum+=(msg->data[i])/1000.0;
}
avg=sum/60.0;
std::cout<<"The average distance in metres is :"<<avg<<std::endl;

//determine if we should set the door detected variable to true or not.
std_msgs::Bool door_detected;
if(avg<=door_threshold){
door_detected.data=true;
}
else{
door_detected.data=false;
}
//publish the door detected variable.
pub.publish(door_detected);

}


int main(int argc, char**argv){

//run the ros init function.
ros::init(argc,argv,"door_detector");
//run the ros nodehandle.
ros::NodeHandle n;

if(n.getParam("/door_detector_node/door_threshold",door_threshold)){
std::cout<<"parameter of door_threshold loaded! "<<door_threshold<<std::endl;
ros::Subscriber sub=n.subscribe("laserscan_raw",20,lasercb);

pub=n.advertise<std_msgs::Bool>("door_detected",20);


ros::spin();
}
else{
std::cout<<"The door threshold parameter could not be loaded."<<std::endl;
}
//std::cin.get();
return 0;
}
