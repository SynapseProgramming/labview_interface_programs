/*
The main function of the lidar interface program is to convert the raw_laserscan data to the sensor_msgs laser_scan message type.
This program is designed to work with the SICK LMS111 Lidar.
*/
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "labview_interface_programs/RawLaser.h"
#include <iostream>
//important constants

//angle_min and angle_max are referenced to the x axis of the robot
//angle_min refers to the angle which corresponds to the first distance point. In the case of the LMS111, it is -134.5 degrees, which equates to -2.347... radians.
const float angle_min=-2.347467844;
//angle_max refers to the angle corresponding to the last distance point. In the case of the LMS111, it is 135.5 degrees, which equates to 2.364... radians
const float angle_max=2.364921136;
//angle_increment refers to the angular difference between distance points. In ths case of the LMS111, it is 0.5 degrees, which equates to 0.00872... radians
const float angle_increment=0.00872664626;
//time_increment refers to the time taken to measure a single distance point. In the case of the LMS111 scanning at a frequency of 10Hz,
//the time for a single distance point measurement is 0.000185 seconds
const float time_increment=0.0001851851852;
//scan_time refers to the time inbetween laserscans in seconds. In the case of the LMS111, it is 0.1 seconds.
const float scan_time=0.1;
//range_min refers to the minimum distance value the lidar can detect in metres. In the case of the LMS111, it is 0.022 metres
const float range_min=0.022;
//range_max refers to the maximum distance value the lidar can detect in metres. In the case of the LMS111, it is 20 metres.
const float range_max=20;

// This is the publisher object
ros::Publisher pub;

void callback(const labview_interface_programs::RawLaser::ConstPtr &msg){
// create the laserscan variable
sensor_msgs::LaserScan scan;
//obtain the time at scan
ros::Time time_at_scan=ros::Time::now();
//populate the laserscan variable with data.
scan.ranges.resize(541);
scan.header.stamp=time_at_scan;
scan.header.frame_id="base_laser";
scan.angle_min=angle_min;
scan.angle_max=angle_max;
scan.angle_increment=angle_increment;
scan.scan_time=scan_time;
scan.range_min=range_min;
scan.range_max=range_max;
/*
For the laserscan ranges[] variable, the first element ranges[0] corresponds to the distance measurement at the minimum angle (-134.5 degrees) in metres
The second element ranges[1] corresponds to the distance measurement at the angle (-134.0 degrees) in metres. and so on...
The last element ranges[540] corresponds to the distance measurement at the angle (135.5 degrees) in metres.
*/

//copy the distance point measurements from the laserscan_raw  to laserscan.
for(unsigned int i=0; i<541;i++){
//we need to divide by 1000 to convert from (mm) to (m)
scan.ranges[i]=(msg->data[i])/1000.0;
}
// publish the laserscan
pub.publish(scan);
//std::cout<<"Publishing laser_scan data on topic: laser_scan"<<std::endl;
}
int main(int argc, char**argv){

//run the ros init function.
ros::init(argc,argv,"lidar_labview_ros_converter");

//create the nodehandle.
ros::NodeHandle n;
//create a subscriber object
ros::Subscriber sub=n.subscribe("laserscan_raw",20,callback);
//define the publisher object.
pub=n.advertise<sensor_msgs::LaserScan>("laser_scan",20);

ros::spin();

return 0;

}
