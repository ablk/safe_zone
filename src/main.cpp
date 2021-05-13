#include <ros/ros.h>
#include <chrono>
#include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>
#include "SafeZone.h"
#include "LaserScanTF.h"

SafeZone safe_zone;
ros::Publisher grid_pub;
ros::Publisher velocity_pub;


void OdomCallback(const nav_msgs::Odometry& odom_msg)
{
	double linear_velocity = odom_msg.twist.twist.linear.x;
	double angular_velocity = odom_msg.twist.twist.angular.z;
	
	
	//avoid noise near zero velocity
	if(fabs(linear_velocity)<0.01)linear_velocity = 0;
	if(fabs(angular_velocity)<0.01)angular_velocity = 0;
	
	
	safe_zone.ComputeZone(linear_velocity,angular_velocity);
	
}

void publish_grid(const ros::TimerEvent& event)
{
	grid_pub.publish(safe_zone.GridMsg());
}


void publish_velocity(const ros::TimerEvent& event){
	
	geometry_msgs::Twist msg = safe_zone.TwistMsg();
	
	//safe_zone.PrintState();
	
	
	if(safe_zone.GetState()!= SAFE_STATE::NORMAL){
		velocity_pub.publish(msg);
	}
	
	
}

int main(int argc, char** argv)
{	
	ros::init(argc, argv, "sz");
	
	ros::NodeHandle nh;
	
	std::string odom_topic = ros::param::param<std::string>("~odom_topic","odom");
	std::string front_scan_topic = ros::param::param<std::string>("~laser_topic1","front_scan");
	std::string back_scan_topic = ros::param::param<std::string>("~laser_topic2","back_scan");
	std::string base_frame = ros::param::param<std::string>("~base_frame","base_footprint");
	std::string output_grid_topic = ros::param::param<std::string>("~output_grid_topic","safe_zone");	
	
	std::string output_velocity_topic = ros::param::param<std::string>("~output_velocity_topic","safe_zone_vel");
	double velocity_publish_duration = ros::param::param<double>("~velocity_publish_duration",1.0/30.0);
	double grid_publish_duration = ros::param::param<double>("~grid_publish_duration",1.0/10.0);
	
	
	double mat_resolution = ros::param::param<double>("~mat_resolution",0.05);
	// zone resolution
	double min_rotate_angle = ros::param::param<double>("~min_rotate_angle",0.10);
	// if rotate too much, we draw a curve rather than a line
	
	double robot_width = ros::param::param<double>("~robot_width",1.5);
	double robot_height = ros::param::param<double>("~robot_height",2);
	// robot size
	
	double trajectory_predict_time = ros::param::param<double>("~trajectory_predict_time",3);
	// slow region size = trajectory_predict_time*odom_speed
	
	double max_speed = ros::param::param<double>("~max_speed",2);
	// robot max speed
	
	double stop_zone_front_dist = ros::param::param<double>("~stop_zone_front_dist",2);
	// stop region size
	
	ros::Subscriber subodom = nh.subscribe(odom_topic, 100, OdomCallback);
	grid_pub = nh.advertise<nav_msgs::OccupancyGrid>(output_grid_topic, 10);
	velocity_pub = nh.advertise<geometry_msgs::Twist>(output_velocity_topic, 1);
	
	//subscriber for front laser
	LaserScanTF ltf_front(
		nh,
		front_scan_topic,
		base_frame,
	&safe_zone);
	
	//subscriber for back laser
	LaserScanTF ltf_back(
		nh,
		back_scan_topic,
		base_frame,
	&safe_zone);
    
	safe_zone.SetParam(
		mat_resolution,
		min_rotate_angle,
		robot_width,
		robot_height,
		trajectory_predict_time,
		max_speed,
		stop_zone_front_dist,
		base_frame
	);
	
	ros::Timer timer = nh.createTimer(ros::Duration(grid_publish_duration), publish_grid);
	ros::Timer timer2 = nh.createTimer(ros::Duration(velocity_publish_duration),publish_velocity);
	ros::spin();
	
	return 0;
}
