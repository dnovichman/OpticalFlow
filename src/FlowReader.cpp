#include <FlowReader.h>

FlowReader::FlowReader() :
ouput_rate(-1)
{
	quality = -1;
	image_height = 480;
	image_width = 640;
	num_feat = 600;
	conf_multi = 20.2f;

	curr_odom_x = 0.0f;
	curr_odom_y = 0.0f;

	frame = "camera";
	altitude = 0.0f;

	frame_time_set = false;
	gyro_x = 0.0f;
	gyro_y = 0.0f;

	focal_length_x = 671.79043f;
	focal_length_y = 672.55843f;
	att_q.x = 0.0f;
	att_q.y = 0.0f;
	att_q.z = 0.0f;
	att_q.w = 1.0f;

	R_imu_in_body.setIdentity();
	R_cam_in_imu.setIdentity();
	camera_offset.setZero();

	prev_img_time = ros::Time::now().toSec();
	imu_timeout = 2.0;
	last_imu_time = ros::Time::now().toSec();

	range_timeout = 2.0;
	last_range_time = ros::Time::now().toSec();
}

FlowReader::~FlowReader() {}

bool FlowReader::initialize(const ros::NodeHandle& n)
{
	node_name = ros::names::append(n.getNamespace(), "FlowReader");

	if (!loadParameters(n))
	{
		ROS_ERROR("%s: initialize: failed to load parameters", node_name.c_str());
		return false;
	}

	if (!registerCallbacks(n))
	{
		ROS_ERROR("%s: initialize: failed to register callbacks", node_name.c_str());
		return false;
	}		
	optical_flow = new OpticalFlowOpenCV((int)focal_length_x, (int)focal_length_y, ouput_rate, image_width, image_height,num_feat, conf_multi);

	return true;
}

bool FlowReader::loadParameters(const ros::NodeHandle& n)
{
	std::vector<float> camera_calib_matrix = {0.0f}, imu_in_body = {0.0f}, cam_in_imu = {0.0f};
	std::vector<float> cam_offset = {0.0f};

	n.getParam("Output_Rate",ouput_rate);
	n.getParam("image_width", image_width);
	n.getParam("image_height", image_height);
	n.getParam("Number_Of_Features", num_feat);

	n.getParam("camera_matrix/data",camera_calib_matrix);	
	n.getParam("cam_offset",cam_offset);
	n.getParam("imu_in_body",imu_in_body);
	n.getParam("cam_in_imu",cam_in_imu);

	n.getParam("Imu_Timeout",imu_timeout);
	n.getParam("Range_Timeout",range_timeout);
	
	R_imu_in_body.setValue(imu_in_body[0], imu_in_body[1], imu_in_body[2], 
							imu_in_body[3], imu_in_body[4], imu_in_body[5], 
							imu_in_body[6], imu_in_body[7], imu_in_body[8]);

	R_cam_in_imu.setValue(cam_in_imu[0], cam_in_imu[1], cam_in_imu[2], 
							cam_in_imu[3], cam_in_imu[4], cam_in_imu[5], 
							cam_in_imu[6], cam_in_imu[7], cam_in_imu[8]);


	focal_length_x = camera_calib_matrix[0];
	focal_length_y = camera_calib_matrix[4];

	camera_offset.setX(cam_offset[0]);
	camera_offset.setY(cam_offset[1]);
	camera_offset.setZ(cam_offset[2]);

  	return true;
}

void FlowReader::imagesCallback(const ros::MessageEvent<sensor_msgs::Image const>& event)
{
	sensor_msgs::Image::ConstPtr msg = event.getMessage();

	if (!frame_time_set)
	{
		first_frame_time = msg->header.stamp.toSec();
		frame_time_set = true;
		prev_img_time = msg->header.stamp.toSec();
	}

	range_dt = ros::Time::now().toSec() - last_range_time;
	if (range_dt > range_timeout)
		return;
	
	imu_dt = ros::Time::now().toSec() - last_imu_time;
	if (imu_dt > imu_timeout)
	{
		gyro_x = 0.0f;
		gyro_y = 0.0f;
	}

	// TODO provide support for other encodings
	std::string img_encoding = "mono8";
	if (img_encoding.compare(msg->encoding))
		return;

	double frame_time = msg->header.stamp.toSec();
	image_height = msg->height;
	image_width = msg->width;
	
	float flow_x_ang = 0.0f;
  	float flow_y_ang = 0.0f;

	frame_time_us = (frame_time - first_frame_time)*1e3f;

	std::copy(std::begin(msg->data), std::end(msg->data), std::begin(image));

	quality = optical_flow->calcFlow((uchar*)image, frame_time_us, dt_us, flow_x_ang, flow_y_ang);

	// Please check gyro directions
	double image_dt = (msg->header.stamp.toSec() - prev_img_time);
	flow_x_ang = (flow_x_ang - gyro_x*image_dt)*altitude;
	flow_y_ang = (flow_y_ang - gyro_x*image_dt)*altitude;

	if (quality > 0)
	{
		geometry_msgs::TwistStamped flow_data;
		flow_data.header = msg->header;
		flow_data.twist.linear.x = flow_x_ang;
		flow_data.twist.linear.y = flow_y_ang;
		flow_pub.publish(flow_data);
		
		update_odom(msg->header, flow_x_ang, flow_y_ang);
	}
	prev_img_time = msg->header.stamp.toSec();
}

void FlowReader::update_odom(std_msgs::Header h, float x, float y)
{
	nav_msgs::Odometry cur_odom;
	cur_odom.header = h;
	cur_odom.header.frame_id = frame;
	curr_odom_x += x;
	curr_odom_y += y;

	// Perform rotations
	tf::Vector3 pose(curr_odom_x, curr_odom_y, altitude);
	tf::Vector3 pose_rot = R_imu_in_body*R_cam_in_imu*pose - camera_offset;

	cur_odom.pose.pose.position.x = pose_rot.getX();
	cur_odom.pose.pose.position.y = pose_rot.getY();
	cur_odom.pose.pose.position.z = pose_rot.getZ();
	
	cur_odom.pose.pose.orientation.x = 0.0f;
	cur_odom.pose.pose.orientation.y = 0.0f;
	cur_odom.pose.pose.orientation.z = 0.0f;
	cur_odom.pose.pose.orientation.w = 1.0f;

	cur_odom.pose.pose.orientation = att_q;
	odom_pub.publish(cur_odom);

	float vx = x/dt_us*1e3f;
	float vy = y/dt_us*1e3f;
}

void FlowReader::imuCallback(const ros::MessageEvent<sensor_msgs::Imu const>& event)
{
	sensor_msgs::Imu::ConstPtr msg = event.getMessage();
	tf::Vector3 gyros(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
	tf::Vector3 gyros_in_cam = R_cam_in_imu.transpose()*gyros;
	gyro_x = gyros_in_cam.getX();
	gyro_y = gyros_in_cam.getY();

	att_q = msg->orientation;
	last_imu_time = ros::Time::now().toSec();
}

void FlowReader::rangeCallback(const ros::MessageEvent<sensor_msgs::Range const>& event)
{
	sensor_msgs::Range::ConstPtr msg = event.getMessage();
	altitude = msg->range;
	last_range_time = ros::Time::now().toSec();
	// We can use float d = agl() * cosf(euler.phi()) * cosf(euler.theta()); to correct if we choose to use sonar or baro
}

bool FlowReader::registerCallbacks(const ros::NodeHandle& n)
{
	ros::NodeHandle optical_node(n);
	images_sub = optical_node.subscribe("images", 1000, &FlowReader::imagesCallback, this);
	imu_sub = optical_node.subscribe("imu", 3, &FlowReader::imuCallback, this);
	range_sub = optical_node.subscribe("range", 3, &FlowReader::rangeCallback, this);

	flow_pub = optical_node.advertise<geometry_msgs::TwistStamped>("twists", 1000, false);
	odom_pub = optical_node.advertise<nav_msgs::Odometry>("odom", 1000, false);
  	return true;
}
