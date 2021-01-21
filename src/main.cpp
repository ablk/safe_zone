#include <ros/ros.h>
#include <chrono>
#include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

#include "SafeZone.h"
#include "LaserScanTF.h"
/*
	g
	h
*/


SafeZone safe_zone;
ros::Publisher grid_pub;

void OdomCallback(const nav_msgs::Odometry& odom_msg)
{
	double linear_velocity = odom_msg.twist.twist.linear.x;
	double angular_velocity = odom_msg.twist.twist.angular.z;
	
	//std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();	
	safe_zone.ComputeZone(linear_velocity,angular_velocity);
	//std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
	//std::cout << "odom:"<<std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()<< "\n";			
}

void publish_grid(const ros::TimerEvent& event)
{
	grid_pub.publish(safe_zone.GridMsg());
}

int main(int argc, char** argv)
{	
	ros::init(argc, argv, "sz");
	
	ros::NodeHandle nh;
	
	ros::Subscriber subodom = nh.subscribe("odom", 100, OdomCallback);
	
	grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("safe_zone", 100);
	
	std::string base_frame = "base_footprint";
	std::string front_scan_topic = "front_scan";
	std::string back_scan_topic = "back_scan";
	
	LaserScanTF ltf_front(
		nh,
		front_scan_topic,
		base_frame,
	&safe_zone);
	
	LaserScanTF ltf_back(
		nh,
		back_scan_topic,
		base_frame,
	&safe_zone);
	
	safe_zone.SetParam(
		0.05,//double mat_resolution,
		1,//double robot_width,
		2,//double robot_height,
		3,//double trajectory_predict_time,
		2,//double max_speed,
		0.25,//double stop_zone_dist,
		0.5,//double slow_zone_dist,
		1.5,//double slow_zone_gain,
		base_frame//std::string frame_id
	);
	
	//TODO:: Opencv draw function have problem when width>height
	
	
	ros::Timer timer = nh.createTimer(ros::Duration(0.1), publish_grid);
	ros::spin();
	
	return 0;
}