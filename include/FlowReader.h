#ifndef FLOW_READER
#define FLOW_READER

#include <ros/ros.h>
#include <string>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include "flow_opencv.hpp"
#include <tf/transform_datatypes.h>

class FlowReader
{
public:
	FlowReader();
	~FlowReader();
	bool initialize(const ros::NodeHandle& n);
private:
	bool loadParameters(const ros::NodeHandle& n);
	bool registerCallbacks(const ros::NodeHandle& n);
	void imagesCallback(const ros::MessageEvent<sensor_msgs::Image const>& event);
	void imuCallback(const ros::MessageEvent<sensor_msgs::Imu const>& event);
	void update_odom(std_msgs::Header h, float x, float y);
	void rangeCallback(const ros::MessageEvent<sensor_msgs::Range const>& event);
	OpticalFlowOpenCV *optical_flow;

	//Publishers
	ros::Publisher flow_pub, odom_pub;
	ros::Subscriber imu_sub, images_sub, range_sub;

	std::string node_name;

	uchar image[921600];
	
	double first_frame_time;
	int dt_us;
	uint32_t frame_time_us;
	int quality, ouput_rate, image_width, image_height, num_feat, step;
	float conf_multi;

	double focal_length_x, focal_length_y;
	double prev_img_time;

	float curr_odom_x, curr_odom_y; 
	std::string frame;
	float altitude;
	bool frame_time_set;

	float gyro_x, gyro_y;
	geometry_msgs::Quaternion att_q;

	std::vector<float> cam_offset;
	tf::Matrix3x3 R_imu_in_body, R_cam_in_imu;
	std::string cam_type;
};
#endif
