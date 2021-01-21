#ifndef _LASERSCANTF_H_
#define _LASERSCANTF_H_ 

#include "SafeZone.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

class LaserScanTF{
	private:
	double angle_min_;
	double angle_max_;
	double range_min_;
	double range_max_;
	double angle_increment_;
	bool is_init_;
	std::string base_frame_;
	
	ros::Subscriber laser_sub;
	
	SafeZone* safe_zone_;
	
	cv::Mat tf_rotation_;
	cv::Mat tf_translation_;
	tf::TransformListener listener;	
	
	public:
	
	LaserScanTF(
		ros::NodeHandle &nh, 
		std::string topic,
		std::string base_frame,
	SafeZone* safe_zone)
	{
		tf_rotation_ = cv::Mat::zeros(3,3,CV_32FC1);
		tf_translation_ = cv::Mat::zeros(3,1,CV_32FC1);
		is_init_ = false;
		laser_sub = nh.subscribe(topic,100,&LaserScanTF::LaserScanCallback, this );
		safe_zone_ = safe_zone;
		base_frame_ = base_frame;
	}
	
	void SetTransform(const tf::StampedTransform& transform){
		tf_rotation_.at<float>(0,0) = transform.getBasis()[0][0];
		tf_rotation_.at<float>(1,0) = transform.getBasis()[1][0];
		tf_rotation_.at<float>(2,0) = transform.getBasis()[2][0];
		tf_rotation_.at<float>(0,1) = transform.getBasis()[0][1];
		tf_rotation_.at<float>(1,1) = transform.getBasis()[1][1];
		tf_rotation_.at<float>(2,1) = transform.getBasis()[2][1];
		tf_rotation_.at<float>(0,2) = transform.getBasis()[0][2];
		tf_rotation_.at<float>(1,2) = transform.getBasis()[1][2];
		tf_rotation_.at<float>(2,2) = transform.getBasis()[2][2];
		tf_translation_.at<float>(0,0) =  transform.getOrigin()[0];
		tf_translation_.at<float>(1,0) =  transform.getOrigin()[1];
		tf_translation_.at<float>(2,0) =  transform.getOrigin()[2];	
		//std::cout<< tf_rotation_<<"\n";
		//std::cout<< tf_translation_<<"\n";
	}
	
	
	
	
	
	cv::Mat Laser2Cloud(const sensor_msgs::LaserScan& laser_msg){
		
		cv::Mat cloud = cv::Mat::zeros(3,laser_msg.ranges.size(),CV_32FC1);
		
		for(int i=0;i<laser_msg.ranges.size();i++){
			double r = laser_msg.ranges[i];
			if(range_min_>r || range_max_<r)continue;
			double angle = angle_min_+angle_increment_*i;
			cloud.at<float>(0,i)=r*cos(angle);
			cloud.at<float>(1,i)=r*sin(angle);
		}
		
		cloud = tf_rotation_*cloud + cv::repeat(tf_translation_,1,laser_msg.ranges.size());
		return cloud;
	}	
	
	void LaserScanCallback(const sensor_msgs::LaserScan& laser_msg){
		if(!is_init_){
			try{
				tf::StampedTransform transform;
				listener.lookupTransform(base_frame_, laser_msg.header.frame_id, 
				ros::Time(0), transform);
				SetTransform(transform);
				
				angle_min_ = laser_msg.angle_min;
				angle_max_ = laser_msg.angle_max;
				angle_increment_ = laser_msg.angle_increment;
				range_min_ = laser_msg.range_min;
				range_max_ = laser_msg.range_max;
				is_init_ = true;
			}
			catch (tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
				ros::Duration(1.0).sleep();
				return;
			}
		}
		
		//std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();	
		
		cv::Mat cloud = Laser2Cloud(laser_msg);
		safe_zone_->DrawCloud(cloud);
		//std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
		//std::cout << "laser:"<<std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()<< "\n";				
		
	}
};

#endif
