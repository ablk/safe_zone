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
	double min_rotate_angle_; //radian	
	double trajectory_predict_time_; //second
	double max_speed_;
	double robot_width_;
	double robot_height_;
	double stop_zone_front_dist_;
	
	int mat_size_;
	int origin_;
	cv::Mat zone_mat_;
	cv::Mat output_zone_mat_;
	int stop_zone_front_pixel_dist_;
	int robot_pixel_height_;
	int robot_pixel_width_;
	std::mutex mutex_;
	
	
	nav_msgs::OccupancyGrid grid_map_;
	bool reset_grid_;
	
	SAFE_STATE state_;
	bool reset_state_;
	geometry_msgs::Twist last_,odom_;
	
	uint8_t min_desired_speed_;
	double scalar_step_;
	
	
	public:
	
	SafeZone(){}
	
	void SetParam(
		double mat_resolution,
		double min_rotate_angle,
		double robot_width,
		double robot_height,
		double trajectory_predict_time,
		double max_speed,
		double stop_zone_front_dist,
		std::string frame_id
		){
		
		//initialize the system
		
		mat_resolution_ = mat_resolution;
		min_rotate_angle_ = min_rotate_angle;
		robot_width_ = robot_width;
		robot_height_ = robot_height;
		trajectory_predict_time_ = trajectory_predict_time;
		max_speed_ = max_speed;
		stop_zone_front_dist_ = stop_zone_front_dist;
		
		
		robot_pixel_width_ = (int)ceil(0.5*robot_width_/mat_resolution_);
		robot_pixel_height_ = (int)ceil(0.5*robot_height_/mat_resolution_);
		origin_ = (int)ceil((stop_zone_front_dist_+trajectory_predict_time_*max_speed_)/mat_resolution_ + std::max(robot_pixel_height_,robot_pixel_width_));
		mat_size_ = origin_*2;
		stop_zone_front_pixel_dist_ = (int)ceil(stop_zone_front_dist_/mat_resolution_);
		
		zone_mat_ = cv::Mat::zeros(mat_size_, mat_size_, CV_8UC1);
		output_zone_mat_ = zone_mat_.clone();        
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
		
		scalar_step_ = 100.0*mat_resolution_/trajectory_predict_time_/max_speed_;		
	}
	
	
	SAFE_STATE GetState(){
		return state_;
	}
	
	void ProcessCloud(const std::vector<cv::Point2f>& cloud){
		// call back function for laser scan
		// project cloud onto safe_zone grid and detect state
		
		const std::lock_guard<std::mutex> lock(mutex_);
		if(reset_state_){
			state_ = SAFE_STATE::NORMAL;
			reset_state_ = false;
			min_desired_speed_ = 0;
		}
		for(int i=0;i<cloud.size();i++){
			float x = cloud[i].x;
			float y = cloud[i].y;
			if(GetValue(x,y)==100){
				state_ = SAFE_STATE::STOP;
				Draw(x,y);			
			}
			if(GetValue(x,y)>0 && GetValue(x,y)<100){
				if(state_ == SAFE_STATE::NORMAL){
					state_ = SAFE_STATE::SLOW;			
				}
				if(state_ == SAFE_STATE::SLOW){
					if(min_desired_speed_ < GetValue(x,y)){
						min_desired_speed_ = GetValue(x,y);
					}
				}
				Draw(x,y);
			}
		}
		
	}
	
	void Draw(float x,float y){
		int u = x/mat_resolution_+origin_;
		int v = y/mat_resolution_+origin_;
		if(u<0||u>=mat_size_||v<0||v>=mat_size_)return;
		output_zone_mat_.at<uint8_t>(v,u)=125;
	}
	
	

	uint8_t GetValue(double x,double y){
		// u = x, v = y;
		//x,y: base_footprint coordinate
		int u = x/mat_resolution_+origin_;
		int v = y/mat_resolution_+origin_;
		if(u<0||u>=mat_size_||v<0||v>=mat_size_)return 0;
		return zone_mat_.at<uint8_t>(v,u);
	}
	
	void ComputeZone(double linear_velocity,double angular_velocity){
		// call back function for odometry
		// draw stop, slow region on safe_zone grid
		// safe : value = 0
		// slow : value = 100~0
		// stop : value = 100
		const std::lock_guard<std::mutex> lock(mutex_);
		odom_.linear.x = linear_velocity;
		odom_.angular.z = angular_velocity;
		
		zone_mat_ = cv::Mat::zeros(mat_size_, mat_size_, CV_8UC1);
		
		double sgn_v = linear_velocity<0? -1:1;
		
		double radius = 0;
		
		if(trajectory_predict_time_*fabs(angular_velocity)>min_rotate_angle_){
			radius = fabs(linear_velocity/angular_velocity/mat_resolution_);
		}
		
		if(radius ==0){
		// slow angular velocity -> draw line	
			std::vector<cv::Point> pts_slow1,pts_slow2;
			double slow_left = origin_- robot_pixel_width_;
			double slow_right = origin_+ robot_pixel_width_;
			double slow_start_distance = origin_ + sgn_v*(robot_pixel_height_ + stop_zone_front_pixel_dist_);
			double slow_end_distance = origin_ + sgn_v*(robot_pixel_height_ + stop_zone_front_pixel_dist_) + linear_velocity*trajectory_predict_time_/mat_resolution_;
			
			for(double d = slow_start_distance;sgn_v*d<sgn_v*slow_end_distance; d = d+sgn_v){
				pts_slow1.push_back(cv::Point(d,slow_left));
				pts_slow2.push_back(cv::Point(d,slow_right));
			}
			pts_slow1.push_back(cv::Point(slow_end_distance,slow_left));
			pts_slow2.push_back(cv::Point(slow_end_distance,slow_right));
			
			for(int i =0 ;i<pts_slow1.size()-1;i++){
				if((pts_slow1[i]==pts_slow1[i+1])&&(pts_slow2[i+1]==pts_slow2[i]))continue;	
				uint8_t scalar = uint8_t(100 - scalar_step_ * (i+1));
				std::vector<cv::Point> pts;
				pts.push_back(pts_slow1[i]);
				pts.push_back(pts_slow1[i+1]);
				pts.push_back(pts_slow2[i+1]);
				pts.push_back(pts_slow2[i]);
				
				fillConvexPoly (zone_mat_,
					pts,
					cv::Scalar(scalar),
					cv::LINE_8,
					0 
				);
			}
			
			std::vector<cv::Point> stop_points;
			stop_points.push_back(cv::Point(origin_ + sgn_v*(robot_pixel_height_ + stop_zone_front_pixel_dist_) ,origin_- robot_pixel_width_));						
			stop_points.push_back(cv::Point(origin_ + sgn_v*(robot_pixel_height_ + stop_zone_front_pixel_dist_) ,origin_+ robot_pixel_width_));	
			stop_points.push_back(cv::Point(origin_ - sgn_v*robot_pixel_height_  ,origin_+ robot_pixel_width_));
			stop_points.push_back(cv::Point(origin_ - sgn_v*robot_pixel_height_  ,origin_-robot_pixel_width_));			
			
			fillConvexPoly (zone_mat_,
				stop_points,
				cv::Scalar(100),
				cv::LINE_8,
				0 
			);
			
		}
		else if(radius-robot_pixel_width_<1){
		// slow linear velocity -> draw circle
			cv::circle(zone_mat_,cv::Point(origin_,origin_), robot_pixel_height_+ robot_pixel_width_, cv::Scalar(100) , -1,cv::LINE_8,0);
		}
		else{
		// 	fast linear & angular velocity -> draw curve	
			double angle_resolution = 1.0/radius;
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
			double slow_radius1 = radius-robot_pixel_width_;
			double slow_radius2 = radius+robot_pixel_width_;
			double slow_start_angle = start_angle + sgn_a*(robot_pixel_height_+stop_zone_front_pixel_dist_) /radius;
			double slow_end_angle = start_angle + sgn_v*angular_velocity*trajectory_predict_time_ + sgn_a*(robot_pixel_height_+stop_zone_front_pixel_dist_) /radius;
			
			for(double angle = slow_start_angle;sgn_a*angle<sgn_a*slow_end_angle;angle+=sgn_a*angle_resolution){
				cv::Point pt1,pt2;
				pt1 = center + cv::Point(slow_radius1*cos(angle),slow_radius1*sin(angle));
				pt2 = center + cv::Point(slow_radius2*cos(angle),slow_radius2*sin(angle));
				pts_slow1.push_back(pt1);
				pts_slow2.push_back(pt2);
			}
			pts_slow1.push_back(center + cv::Point(slow_radius1*cos(slow_end_angle),slow_radius1*sin(slow_end_angle)));
			pts_slow2.push_back(center + cv::Point(slow_radius2*cos(slow_end_angle),slow_radius2*sin(slow_end_angle)));
			
			for(int i =0 ;i<pts_slow1.size()-1;i++){
				if((pts_slow1[i]==pts_slow1[i+1])&&(pts_slow2[i+1]==pts_slow2[i])){
					continue;
				}
				uint8_t scalar = uint8_t(100 -  scalar_step_ * (i+1));
				std::vector<cv::Point> pts;
				pts.push_back(pts_slow1[i]);
				pts.push_back(pts_slow1[i+1]);
				pts.push_back(pts_slow2[i+1]);
				pts.push_back(pts_slow2[i]);
				
				fillConvexPoly (zone_mat_,
					pts,
					cv::Scalar(scalar),
					cv::LINE_8,
					0 
				);
			}			
			
			std::vector<cv::Point> pts_stop1,pts_stop2;
			double stop_radius1 = radius-robot_pixel_width_;
			double stop_radius2 = radius+robot_pixel_width_;
			double stop_start_angle = start_angle - sgn_a*robot_pixel_height_ /radius;
			double stop_end_angle = start_angle + sgn_a*(robot_pixel_height_+stop_zone_front_pixel_dist_) /radius;
			
			for(double angle = stop_start_angle;sgn_a*angle<sgn_a*stop_end_angle;angle+=sgn_a*angle_resolution){
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
	//visualization safe zone
		const std::lock_guard<std::mutex> lock(mutex_);
		output_zone_mat_.reshape(0, 1).copyTo(grid_map_.data);
		reset_grid_ = true;
		return grid_map_;
	}
	
	geometry_msgs::Twist TwistMsg(){
	// out put velocity according to nearest laser on safe_zone grid 
		const std::lock_guard<std::mutex> lock(mutex_);
		if(state_ == SAFE_STATE::SLOW){
			double desired_linear_velocity = max_speed_*(100-min_desired_speed_)/100.0;
			
			if(desired_linear_velocity<abs(odom_.linear.x)){
				double sgn_v = odom_.linear.x>0? 1:-1;
				last_.linear.x = sgn_v*desired_linear_velocity;
				last_.angular.z = odom_.angular.z * desired_linear_velocity / abs(odom_.linear.x);
			}
			else{
				last_.linear.x = odom_.linear.x;
				last_.angular.z = odom_.angular.z;
				state_=SAFE_STATE::NORMAL;
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
		
		reset_state_ = true;
		return last_;
	}
	
	void PrintState(){
		const std::lock_guard<std::mutex> lock(mutex_);
		if(state_ == SAFE_STATE::SLOW){
			std::cout<<"SLOW--\t";
		}
		
		if(state_ == SAFE_STATE::STOP){
			std::cout<<"STOP!!\t";
		}
		if(state_ == SAFE_STATE::NORMAL){
			std::cout<<"NORMAL\t";
		}
		std::cout<<int(min_desired_speed_)<<"("<<odom_.linear.x<<","<<odom_.angular.z<<")("<<last_.linear.x<<","<<last_.angular.z<<")\n";
		
	}
	
	
};
	
#endif

