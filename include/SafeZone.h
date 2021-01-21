#ifndef _SAFEZONE_H_
#define _SAFEZONE_H_ 

#include <nav_msgs/OccupancyGrid.h>
#include <cmath>
#include <mutex>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class SafeZone{
	private:
	double mat_resolution_; //meter
	double trajectory_predict_time_; //second
	double max_speed_;
	double robot_width_;
	double robot_height_;
	double stop_zone_dist_;
	double slow_zone_dist_;
	double slow_zone_gain_;
	
	double mat_size_;
	double origin_;
	cv::Mat zone_mat_;
	double stop_zone_pixel_dist_;
	double slow_zone_pixel_dist_;
	double robot_pixel_height_;
	double robot_pixel_width_;
	std::mutex mutex_;
	
	
	nav_msgs::OccupancyGrid grid_map_;
	
	
	public:	
	
	SafeZone(){
		zone_mat_ = cv::Mat::zeros(mat_size_, mat_size_, CV_8UC1);
		mat_resolution_ = 0.05;
		robot_width_ = 1;
		robot_height_ = 2;
		trajectory_predict_time_ = 3;
		max_speed_ = 2;
		stop_zone_dist_ = 0.5;
		slow_zone_dist_ = 1;
		slow_zone_gain_ = 1.5;
		
		
		robot_pixel_width_ = 0.5*robot_width_/mat_resolution_;
		robot_pixel_height_ = 0.5*robot_height_/mat_resolution_;
		origin_ = trajectory_predict_time_*max_speed_/mat_resolution_ + robot_pixel_height_;
		mat_size_ = origin_*2;
		stop_zone_pixel_dist_ = robot_pixel_width_ + (stop_zone_dist_)/mat_resolution_;
		slow_zone_pixel_dist_ = robot_pixel_width_ + (stop_zone_dist_ + slow_zone_dist_)/mat_resolution_;
		
		
		grid_map_.header.frame_id = "base_footprint";
		grid_map_.info.resolution=mat_resolution_;
		grid_map_.info.origin.position.z=0;
		grid_map_.info.origin.orientation.w=1;
		grid_map_.info.origin.orientation.x=0;
		grid_map_.info.origin.orientation.y=0;
		grid_map_.info.origin.orientation.z=0;
		grid_map_.info.width = mat_size_;
		grid_map_.info.height = mat_size_;
		grid_map_.info.origin.position.x=-origin_*mat_resolution_;
		grid_map_.info.origin.position.y=-origin_*mat_resolution_;
		
	}
	
	void SetParam(
		double mat_resolution,
		double robot_width,
		double robot_height,
		double trajectory_predict_time,
		double max_speed,
		double stop_zone_dist,
		double slow_zone_dist,
		double slow_zone_gain,
		std::string frame_id){
		
		zone_mat_ = cv::Mat::zeros(mat_size_, mat_size_, CV_8UC1);
		mat_resolution_ = mat_resolution;
		robot_width_ = robot_width;
		robot_height_ = robot_height;
		trajectory_predict_time_ = trajectory_predict_time;
		max_speed_ = max_speed;
		stop_zone_dist_ = stop_zone_dist;
		slow_zone_dist_ = slow_zone_dist;
		slow_zone_gain_ = slow_zone_gain;
		
		
		robot_pixel_width_ = 0.5*robot_width_/mat_resolution_;
		robot_pixel_height_ = 0.5*robot_height_/mat_resolution_;
		origin_ = trajectory_predict_time_*max_speed_/mat_resolution_ + robot_pixel_height_;
		mat_size_ = origin_*2;
		stop_zone_pixel_dist_ = stop_zone_dist_/mat_resolution_;
		slow_zone_pixel_dist_ = stop_zone_pixel_dist_ + slow_zone_dist_/mat_resolution_;
		
		
		grid_map_.header.frame_id = frame_id;
		grid_map_.info.resolution=mat_resolution_;
		grid_map_.info.origin.position.z=0;
		grid_map_.info.origin.orientation.w=1;
		grid_map_.info.origin.orientation.x=0;
		grid_map_.info.origin.orientation.y=0;
		grid_map_.info.origin.orientation.z=0;
		grid_map_.info.width = mat_size_;
		grid_map_.info.height = mat_size_;
		grid_map_.info.origin.position.x=-origin_*mat_resolution_;
		grid_map_.info.origin.position.y=-origin_*mat_resolution_;
	}
	
	
	void DrawCloud(cv::Mat cloud){
		const std::lock_guard<std::mutex> lock(mutex_);
		//std::cout<<cloud.rows<<","<<cloud.cols<<"\n";
		for(int i=0;i<cloud.cols;i++){
			float x = cloud.at<float>(0,i);
			float y = cloud.at<float>(1,i);
			Draw(x,y);
		}
		
	}
	
	void Draw(float x,float y){
		int u = x/mat_resolution_+origin_;
		int v = y/mat_resolution_+origin_;
		if(u<0||u>mat_size_||v<0||v>mat_size_)return;
		zone_mat_.at<uint8_t>(v,u)=125;
	}
	
	
	// u = x, v = y;
	uint8_t GetValue(double x,double y){
		//x,y: base_footprint coordinate
		int u = x/mat_resolution_+origin_;
		int v = y/mat_resolution_+origin_;
		return zone_mat_.at<uint8_t>(v,u);
	}
	
	void ComputeZone(double linear_velocity,double angular_velocity){
		const std::lock_guard<std::mutex> lock(mutex_);
		zone_mat_ = cv::Mat::zeros(mat_size_, mat_size_, CV_8UC1);
		double sgn_v = linear_velocity<0? -1:1;
		
		double radius = 0;
		if(fabs(angular_velocity)>0.05){
			radius = fabs(linear_velocity/angular_velocity/mat_resolution_);
		}
		
		if(radius > mat_size_ || radius ==0){
			cv::line(zone_mat_, cv::Point(origin_ - sgn_v*robot_pixel_height_ ,origin_), cv::Point(origin_ + sgn_v*robot_pixel_height_ + slow_zone_gain_*linear_velocity*trajectory_predict_time_/mat_resolution_,origin_), cv::Scalar(50), (slow_zone_pixel_dist_ + robot_pixel_width_)*2);
			cv::line(zone_mat_, cv::Point(origin_ - sgn_v*robot_pixel_height_,origin_), cv::Point(origin_ + sgn_v*robot_pixel_height_ + linear_velocity*trajectory_predict_time_/mat_resolution_,origin_), cv::Scalar(100), (stop_zone_pixel_dist_ + robot_pixel_width_)*2);	
		}
		else if(radius<1/mat_resolution_){
			cv::circle(zone_mat_,cv::Point(origin_,origin_), slow_zone_pixel_dist_+std::max(robot_pixel_height_,robot_pixel_width_), cv::Scalar(50) , -1,cv::LINE_4,0);
			cv::circle(zone_mat_,cv::Point(origin_,origin_), stop_zone_pixel_dist_+std::max(robot_pixel_height_,robot_pixel_width_), cv::Scalar(100) , -1,cv::LINE_4,0);				
		}
		else{
			cv::Point center;
			double start_angle;
			double sgn_a;
			if(angular_velocity<0){
				center = cv::Point(origin_,origin_-radius);
				start_angle = 90;
				sgn_a = -sgn_v;
			}
			else{
				center = cv::Point(origin_,origin_+radius);
				start_angle = 270;
				sgn_a = sgn_v;
			}
			
			cv::ellipse(zone_mat_,
				center,
				cv::Size(radius,radius),
				0,
				start_angle - sgn_a*robot_pixel_height_/radius*180.0/M_PI,
				start_angle+slow_zone_gain_*sgn_v*angular_velocity*trajectory_predict_time_*180.0/M_PI + sgn_a*robot_pixel_height_/radius*180.0/M_PI,
				cv::Scalar(50),
				(slow_zone_pixel_dist_+robot_pixel_width_)*2,
				cv::LINE_4,
				0
			);
			
			
			cv::ellipse(zone_mat_,
				center,
				cv::Size(radius,radius),
				0,
				start_angle - sgn_a*robot_pixel_height_/radius*180.0/M_PI,
				start_angle+sgn_v*angular_velocity*trajectory_predict_time_*180.0/M_PI + sgn_a*robot_pixel_height_/radius*180.0/M_PI,
				cv::Scalar(100),
				(stop_zone_pixel_dist_+robot_pixel_width_)*2,
				cv::LINE_4,
				0
			);
		}	
	}
	
	nav_msgs::OccupancyGrid GridMsg(){
		const std::lock_guard<std::mutex> lock(mutex_);
		cv::Mat tmp = zone_mat_.reshape(0, 1);
		tmp.copyTo(grid_map_.data);
		return grid_map_;
	}
};

#endif

