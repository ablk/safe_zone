#ifndef _SAFEZONE_H_
#define _SAFEZONE_H_ 

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <mutex>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


enum class SAFE_STATE{
	NORMAL,
	SLOW,
	STOP
};

class SafeZone{
	private:
	double mat_resolution_; //meter
	double angle_resolution_; //radius	
	double trajectory_predict_time_; //second
	double max_speed_;
	double robot_width_;
	double robot_height_;
	double stop_zone_dist_;
	double slow_zone_dist_;
	double slow_zone_gain_;

	
	int mat_size_;
	int origin_;
	cv::Mat zone_mat_;
	cv::Mat output_zone_mat_;
	int stop_zone_pixel_dist_;
	int slow_zone_pixel_dist_;
	int robot_pixel_height_;
	int robot_pixel_width_;
	std::mutex mutex_;
	
	
	nav_msgs::OccupancyGrid grid_map_;
	bool reset_grid_;

	SAFE_STATE state_;
	bool reset_state_;
	geometry_msgs::Twist last_,odom_;
	double desired_linear_velocity_,desired_angular_velocity_;
	double max_acc_x_,max_acc_w_;
	double velocity_publish_duration_;
	
	
	public:
	
	SafeZone(){
		zone_mat_ = cv::Mat::zeros(mat_size_, mat_size_, CV_8UC1);
		output_zone_mat_ = zone_mat_.clone();
		mat_resolution_ = 0.05;
		angle_resolution_ = 10.0 * M_PI/180.0;
		robot_width_ = 1;
		robot_height_ = 2;
		trajectory_predict_time_ = 3;
		max_speed_ = 2;
		stop_zone_dist_ = 0.5;
		slow_zone_dist_ = 1;
		slow_zone_gain_ = 1.5;
		
		robot_pixel_width_ = (int)ceil(0.5*robot_width_/mat_resolution_);
		robot_pixel_height_ = (int)ceil(0.5*robot_height_/mat_resolution_);
		origin_ = (int)ceil(trajectory_predict_time_*max_speed_/mat_resolution_ + sqrt(robot_pixel_height_*robot_pixel_height_+robot_pixel_width_*robot_pixel_width_));
		mat_size_ = origin_*2;
		stop_zone_pixel_dist_ = (int)ceil(stop_zone_dist_/mat_resolution_);
		slow_zone_pixel_dist_ = (int)ceil(stop_zone_pixel_dist_ + slow_zone_dist_/mat_resolution_);
		
		
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

		state_ = SAFE_STATE::NORMAL;
		reset_grid_ = true;
		reset_state_ = true;
		
		
		desired_linear_velocity_ = 0;
		desired_angular_velocity_ = 0;
		max_acc_x_ = 1;
		max_acc_w_ = 3;
		velocity_publish_duration_ = 1.0/30.0;
	}
	
	void SetParam(
		double mat_resolution,
		double angle_resolution,
		double robot_width,
		double robot_height,
		double trajectory_predict_time,
		double max_speed,
		double stop_zone_dist,
		double slow_zone_dist,
		double slow_zone_gain,
		std::string frame_id,
		double desired_linear_velocity,
		double desired_angular_velocity,
		double	max_acc_x,
		double max_acc_w,
		double velocity_publish_duration
		){
		
		zone_mat_ = cv::Mat::zeros(mat_size_, mat_size_, CV_8UC1);
		output_zone_mat_ = zone_mat_.clone();
		mat_resolution_ = mat_resolution;
		angle_resolution_ = angle_resolution;
		robot_width_ = robot_width;
		robot_height_ = robot_height;
		trajectory_predict_time_ = trajectory_predict_time;
		max_speed_ = max_speed;
		stop_zone_dist_ = stop_zone_dist;
		slow_zone_dist_ = slow_zone_dist;
		slow_zone_gain_ = slow_zone_gain;
		
		
		robot_pixel_width_ = (int)ceil(0.5*robot_width_/mat_resolution_);
		robot_pixel_height_ = (int)ceil(0.5*robot_height_/mat_resolution_);
		origin_ = (int)ceil(trajectory_predict_time_*max_speed_/mat_resolution_ + sqrt(robot_pixel_height_*robot_pixel_height_+robot_pixel_width_*robot_pixel_width_));
		mat_size_ = origin_*2;
		stop_zone_pixel_dist_ = (int)ceil(stop_zone_dist_/mat_resolution_);
		slow_zone_pixel_dist_ = (int)ceil(stop_zone_pixel_dist_ + slow_zone_dist_/mat_resolution_);
	
		
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
		state_ = SAFE_STATE::NORMAL;
		reset_grid_ = true;
		reset_state_ = true;
		
		desired_linear_velocity_ = desired_linear_velocity;
		desired_angular_velocity_ = desired_angular_velocity;
		max_acc_x_ = max_acc_x;
		max_acc_w_ = max_acc_w;
		velocity_publish_duration_ = velocity_publish_duration;		
	}
	
	
	SAFE_STATE GetState(){
		return state_;
	}
	
	void DrawCloud(cv::Mat cloud){
		const std::lock_guard<std::mutex> lock(mutex_);
		//std::cout<<cloud.rows<<","<<cloud.cols<<"\n";

		if(reset_state_){
			state_ = SAFE_STATE::NORMAL;
			reset_state_ = false;		
		}
		for(int i=0;i<cloud.cols;i++){
			float x = cloud.at<float>(0,i);
			float y = cloud.at<float>(1,i);
			if(GetValue(x,y)==100){
				state_ = SAFE_STATE::STOP;
				Draw(x,y);			
			}
			if(GetValue(x,y)==50){
				Draw(x,y);
				if(state_ == SAFE_STATE::NORMAL){
					state_ = SAFE_STATE::SLOW;			
				}		
			}
		}
		
	}
	
	void Draw(float x,float y){
		int u = x/mat_resolution_+origin_;
		int v = y/mat_resolution_+origin_;
		if(u<0||u>=mat_size_||v<0||v>=mat_size_)return;
		output_zone_mat_.at<uint8_t>(v,u)=125;
	}
	
	
	// u = x, v = y;
	uint8_t GetValue(double x,double y){
		//x,y: base_footprint coordinate
		int u = x/mat_resolution_+origin_;
		int v = y/mat_resolution_+origin_;
		if(u<0||u>=mat_size_||v<0||v>=mat_size_)return 0;
		return zone_mat_.at<uint8_t>(v,u);
	}
	
	void ComputeZone(double linear_velocity,double angular_velocity){
		const std::lock_guard<std::mutex> lock(mutex_);
		odom_.linear.x = linear_velocity;
		odom_.angular.z = angular_velocity;
		
		zone_mat_ = cv::Mat::zeros(mat_size_, mat_size_, CV_8UC1);

		double sgn_v = linear_velocity<0? -1:1;
		
		double radius = 0;
		
		if(trajectory_predict_time_*fabs(angular_velocity)>angle_resolution_){
			radius = fabs(linear_velocity/angular_velocity/mat_resolution_);
		}

		if(radius ==0){
			std::vector<cv::Point> slow_points;
			slow_points.push_back(cv::Point(origin_ + sgn_v*(robot_pixel_height_ + slow_zone_pixel_dist_) + slow_zone_gain_*linear_velocity*trajectory_predict_time_/mat_resolution_,origin_-(slow_zone_pixel_dist_ + robot_pixel_width_)));						
			slow_points.push_back(cv::Point(origin_ + sgn_v*(robot_pixel_height_ + slow_zone_pixel_dist_) + slow_zone_gain_*linear_velocity*trajectory_predict_time_/mat_resolution_,origin_+(slow_zone_pixel_dist_ + robot_pixel_width_)));	
			slow_points.push_back(cv::Point(origin_ - sgn_v*(robot_pixel_height_ + slow_zone_pixel_dist_) ,origin_+(slow_zone_pixel_dist_ + robot_pixel_width_)));
			slow_points.push_back(cv::Point(origin_ - sgn_v*(robot_pixel_height_ + slow_zone_pixel_dist_) ,origin_-(slow_zone_pixel_dist_ + robot_pixel_width_)));
			
			std::vector<cv::Point> stop_points;
			stop_points.push_back(cv::Point(origin_ + sgn_v*(robot_pixel_height_ + stop_zone_pixel_dist_) + linear_velocity*trajectory_predict_time_/mat_resolution_,origin_-(stop_zone_pixel_dist_ + robot_pixel_width_)));						
			stop_points.push_back(cv::Point(origin_ + sgn_v*(robot_pixel_height_ + stop_zone_pixel_dist_) + linear_velocity*trajectory_predict_time_/mat_resolution_,origin_+(stop_zone_pixel_dist_ + robot_pixel_width_)));	
			stop_points.push_back(cv::Point(origin_ - sgn_v*(robot_pixel_height_ + stop_zone_pixel_dist_) ,origin_+(stop_zone_pixel_dist_ + robot_pixel_width_)));
			stop_points.push_back(cv::Point(origin_ - sgn_v*(robot_pixel_height_ + stop_zone_pixel_dist_) ,origin_-(stop_zone_pixel_dist_ + robot_pixel_width_)));			

			fillConvexPoly (zone_mat_,
				slow_points,
				cv::Scalar(50),
				cv::LINE_8,
				0 
			);

			fillConvexPoly (zone_mat_,
				stop_points,
				cv::Scalar(100),
				cv::LINE_8,
				0 
			);

		}
		else if(radius-slow_zone_pixel_dist_-robot_pixel_width_<1){
			cv::circle(zone_mat_,cv::Point(origin_,origin_), slow_zone_pixel_dist_+sqrt(robot_pixel_height_*robot_pixel_height_+robot_pixel_width_*robot_pixel_width_), cv::Scalar(50) , -1,cv::LINE_8,0);
			cv::circle(zone_mat_,cv::Point(origin_,origin_), stop_zone_pixel_dist_+sqrt(robot_pixel_height_*robot_pixel_height_+robot_pixel_width_*robot_pixel_width_), cv::Scalar(100) , -1,cv::LINE_8,0);
		}
		else{	
			cv::Point center;
			double start_angle;
			double sgn_a;
			if(angular_velocity<0){
				center = cv::Point(origin_,origin_-radius);
				start_angle = 90.0*M_PI/180.0;
				sgn_a = -sgn_v;
			}
			else{
				center = cv::Point(origin_,origin_+radius);
				start_angle = 270*M_PI/180.0;
				sgn_a = sgn_v;
			}
			
			
			std::vector<cv::Point> pts_slow1,pts_slow2;
			double slow_radius1 = radius-slow_zone_pixel_dist_-robot_pixel_width_;
			double slow_radius2 = radius+slow_zone_pixel_dist_+robot_pixel_width_;
			double slow_start_angle = start_angle - sgn_a*(robot_pixel_height_+slow_zone_pixel_dist_) /radius;
			double slow_end_angle = start_angle + sgn_v*slow_zone_gain_*angular_velocity*trajectory_predict_time_ + sgn_a*(robot_pixel_height_+slow_zone_pixel_dist_) /radius;
	
			
			if(slow_end_angle-slow_start_angle<angle_resolution_){
				double tmp = slow_end_angle;
				slow_end_angle = slow_start_angle;
				slow_start_angle = tmp;
			}
			for(double angle = slow_start_angle;angle<slow_end_angle;angle+=angle_resolution_){
				pts_slow1.push_back(center + cv::Point(slow_radius1*cos(angle),slow_radius1*sin(angle)));
				pts_slow2.push_back(center + cv::Point(slow_radius2*cos(angle),slow_radius2*sin(angle)));
			}
			pts_slow1.push_back(center + cv::Point(slow_radius1*cos(slow_end_angle),slow_radius1*sin(slow_end_angle)));
			pts_slow2.push_back(center + cv::Point(slow_radius2*cos(slow_end_angle),slow_radius2*sin(slow_end_angle)));
			
			for(int i =0 ;i<pts_slow1.size()-1;i++){
				if((pts_slow1[i]==pts_slow1[i+1])&&(pts_slow2[i+1]==pts_slow2[i]))continue;		
				std::vector<cv::Point> pts;
				pts.push_back(pts_slow1[i]);
				pts.push_back(pts_slow1[i+1]);
				pts.push_back(pts_slow2[i+1]);
				pts.push_back(pts_slow2[i]);
			
				fillConvexPoly (zone_mat_,
					pts,
					cv::Scalar(50),
					cv::LINE_8,
					0 
				);
			}			
			
			std::vector<cv::Point> pts_stop1,pts_stop2;
			double stop_radius1 = radius-stop_zone_pixel_dist_-robot_pixel_width_;
			double stop_radius2 = radius+stop_zone_pixel_dist_+robot_pixel_width_;
			double stop_start_angle = start_angle - sgn_a*(robot_pixel_height_+stop_zone_pixel_dist_) /radius;
			double stop_end_angle = start_angle + sgn_v*angular_velocity*trajectory_predict_time_ + sgn_a*(robot_pixel_height_+stop_zone_pixel_dist_) /radius;
	
			
			if(stop_end_angle-stop_start_angle<angle_resolution_){
				double tmp = stop_end_angle;
				stop_end_angle = stop_start_angle;
				stop_start_angle = tmp;
			}
			for(double angle = stop_start_angle;angle<stop_end_angle;angle+=angle_resolution_){
				pts_stop1.push_back(center + cv::Point(stop_radius1*cos(angle),stop_radius1*sin(angle)));
				pts_stop2.push_back(center + cv::Point(stop_radius2*cos(angle),stop_radius2*sin(angle)));
			}
			pts_stop1.push_back(center + cv::Point(stop_radius1*cos(stop_end_angle),stop_radius1*sin(stop_end_angle)));
			pts_stop2.push_back(center + cv::Point(stop_radius2*cos(stop_end_angle),stop_radius2*sin(stop_end_angle)));
			for(int i =0 ;i<pts_stop1.size()-1;i++){
				if((pts_stop1[i]==pts_stop1[i+1])&&(pts_stop2[i+1]==pts_stop2[i]))continue;		
				std::vector<cv::Point> pts;
				pts.push_back(pts_stop1[i]);
				pts.push_back(pts_stop1[i+1]);
				pts.push_back(pts_stop2[i+1]);
				pts.push_back(pts_stop2[i]);
			
				fillConvexPoly (zone_mat_,
					pts,
					cv::Scalar(100),
					cv::LINE_8,
					0 
				);

			}
		}
		if(reset_grid_){

			output_zone_mat_ = zone_mat_.clone();
			reset_grid_ = false;
		}
	}
	
	nav_msgs::OccupancyGrid GridMsg(){
		const std::lock_guard<std::mutex> lock(mutex_);
		output_zone_mat_.reshape(0, 1).copyTo(grid_map_.data);
		reset_grid_ = true;
		return grid_map_;
	}

	geometry_msgs::Twist TwistMsg(){
		const std::lock_guard<std::mutex> lock(mutex_);
		if(state_ == SAFE_STATE::SLOW){
			if(odom_.linear.x>desired_linear_velocity_){
				last_.linear.x = std::max(last_.linear.x,odom_.linear.x);
				last_.linear.x = std::max(desired_linear_velocity_,last_.linear.x - max_acc_x_ * velocity_publish_duration_);
			}
			else if(odom_.linear.x<-desired_linear_velocity_){
				last_.linear.x = std::min(last_.linear.x,odom_.linear.x);
				last_.linear.x = std::min(-desired_linear_velocity_,last_.linear.x + max_acc_x_ * velocity_publish_duration_);
			}
			else{
				last_.linear.x = odom_.linear.x;
				//Handle if state = slow but odom is ok
			}
			
			
			if(odom_.angular.z>desired_angular_velocity_){
				last_.angular.z = std::max(last_.angular.z,odom_.angular.z);
				last_.angular.z = std::max(desired_angular_velocity_,last_.angular.z - max_acc_w_ * velocity_publish_duration_);
			}
			else if(odom_.angular.z<-desired_angular_velocity_){
				last_.angular.z = std::min(last_.angular.z,odom_.angular.z);
				last_.angular.z = std::min(-desired_angular_velocity_,last_.angular.z + max_acc_w_ * velocity_publish_duration_);
			}
			else{
				last_.angular.z = odom_.angular.z;
			}
		}
		if(state_ == SAFE_STATE::STOP){
			last_.linear.x = 0;
			last_.angular.z = 0;
		}
		if(state_==SAFE_STATE::NORMAL){
			last_.linear.x = odom_.linear.x;
			last_.angular.z = odom_.angular.z;
		}		
		
		/*
		if(state_ == SAFE_STATE::SLOW){
			std::cout<<"SLOW";
			}
		if(state_ == SAFE_STATE::STOP){
			std::cout<<"!!!!!STOP!!!!!";
		}
		if(state_ == SAFE_STATE::NORMAL){
			std::cout<<"NORMAL";
		}
		std::cout<<"("<<odom_.linear.x<<","<<odom_.angular.z<<")("<<last_.linear.x<<","<<last_.angular.z<<")\n";
		*/
		
		reset_state_ = true;
		return last_;
	}
	

};

#endif

