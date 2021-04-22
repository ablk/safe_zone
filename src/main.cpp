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


//1. if state = slow but odom is ok
//2. published twist may not on origin nav route
//3.

void OdomCallback(const nav_msgs::Odometry& odom_msg)
{
	double linear_velocity = odom_msg.twist.twist.linear.x;
	double angular_velocity = odom_msg.twist.twist.angular.z;
	
	//std::cout<<"odom:"<<linear_velocity<<","<<angular_velocity<<"\n";
	
	//std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();	
	safe_zone.ComputeZone(linear_velocity,angular_velocity);

	//std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
	//std::cout << "odom:"<<std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()<< "\n";			
}

void publish_grid(const ros::TimerEvent& event)
{
	grid_pub.publish(safe_zone.GridMsg());
}


void publish_velocity(const ros::TimerEvent& event){
	
	geometry_msgs::Twist msg = safe_zone.TwistMsg();
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
	double angle_resolution = ros::param::param<double>("~angle_resolution",0.10);
	// zone resolution
	
	double robot_width = ros::param::param<double>("~robot_width",1.5);
	double robot_height = ros::param::param<double>("~robot_height",2);
	// robot size

	double trajectory_predict_time = ros::param::param<double>("~trajectory_predict_time",3);
	// grow region size = trajectory_predict_time*odom_speed
	
	double max_speed = ros::param::param<double>("~max_speed",2);
	// grid size = trajectory_predict_time*max_speed
	
	double stop_zone_dist = ros::param::param<double>("~stop_zone_dist",0.25);
	double slow_zone_dist = ros::param::param<double>("~slow_zone_dist",0.5);
	// stop_zone_width = robot_width/2 + stop_zone_dist
	// slow_zone_width = robot_width/2 + slow_zone_dist
	
	double slow_zone_gain = ros::param::param<double>("~slow_zone_gain",1.5);
	// slow_zone grow region gain
	
	
	double max_acc_x = ros::param::param<double>("~max_acc_x",1);
	double max_acc_w = ros::param::param<double>("~max_acc_w",1);
	double desired_linear_velocity = ros::param::param<double>("~desired_linear_velocity",0.1);
	double desired_angular_velocity = ros::param::param<double>("~desired_angular_velocity",0.1);
	
	
	ros::Subscriber subodom = nh.subscribe(odom_topic, 100, OdomCallback);
	grid_pub = nh.advertise<nav_msgs::OccupancyGrid>(output_grid_topic, 10);
	velocity_pub = nh.advertise<geometry_msgs::Twist>(output_velocity_topic, 1);
	
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
		mat_resolution,
		angle_resolution,
		robot_width,
		robot_height,
		trajectory_predict_time,
		max_speed,
		stop_zone_dist,
		slow_zone_dist,
		slow_zone_gain,
		base_frame,
		desired_linear_velocity,
		desired_angular_velocity,
		max_acc_x,
		max_acc_w,
		velocity_publish_duration
	);

	
	ros::Timer timer = nh.createTimer(ros::Duration(grid_publish_duration), publish_grid);
	ros::Timer timer2 = nh.createTimer(ros::Duration(velocity_publish_duration),publish_velocity);
	ros::spin();
	
	return 0;
}