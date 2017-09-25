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

	frame = "world";
	altitude = 1.0f;

	frame_time_set = false;
	gyro_x = 0.0f;
	gyro_y = 0.0f;

	focal_length_x = 671.79043f;
	focal_length_y = 672.55843f;
	att_q.x = 0.0f;
	att_q.y = 0.0f;
	att_q.z = 0.0f;
	att_q.w = 1.0f;
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
	// In general, these should be part of camera param file
	n.getParam("Output_Rate",ouput_rate);
	n.getParam("Image_Width", image_width);
	n.getParam("Image_Height", image_height);
	n.getParam("Number_Of_Features", num_feat);

	// Create a yaml file reader for camera parameters
	/*n.getParam("camera_matrix/data[0]",focal_length_x);
	n.getParam("camera_matrix/data[4]",focal_length_y);*/

  	return true;
}

void FlowReader::process_images(const ros::MessageEvent<sensor_msgs::Image const>& event)
{
	sensor_msgs::Image::ConstPtr msg = event.getMessage();

	if (!frame_time_set)
	{
		first_frame_time = msg->header.stamp.toSec();
		frame_time_set = true;
	}

	double frame_time = msg->header.stamp.toSec();
	image_height = msg->height;
	image_width = msg->width;
	
	float flow_x_ang = 0.0f;
  	float flow_y_ang = 0.0f;

	frame_time_us = (frame_time - first_frame_time)*1e3f;

	std::copy(std::begin(msg->data), std::end(msg->data), std::begin(image));

	#if 0
	for (int i=0; i<307200; i++)
	{
		//ROS_INFO("d %u",image[i]); //-msg->data[i]
	i++;
	}
	#endif
	quality = optical_flow->calcFlow((uchar*)image, frame_time_us, dt_us, flow_x_ang, flow_y_ang);
	// Check coordinate system
	flow_x_ang = (flow_x_ang - gyro_x)*altitude;
	flow_y_ang = (flow_y_ang - gyro_y)*altitude;
	#if 0
	ROS_INFO("Image quality is %d %d", quality, dt_us);
	#endif
	if (quality > 0)
	{
		geometry_msgs::TwistStamped flow_data;
		flow_data.header = msg->header;
		flow_data.twist.linear.x = flow_x_ang;
		flow_data.twist.linear.y = flow_y_ang;
		flow_pub.publish(flow_data);
		
		update_odom(msg->header, flow_x_ang, flow_y_ang);
	}
}

void FlowReader::update_odom(std_msgs::Header h, float x, float y)
{
	nav_msgs::Odometry cur_odom;
	cur_odom.header = h;
	cur_odom.header.frame_id = frame;
	curr_odom_x += x;
	curr_odom_y += y;

	cur_odom.pose.pose.position.x = curr_odom_x;
	cur_odom.pose.pose.position.y = curr_odom_y;
	cur_odom.pose.pose.position.z = altitude; // NED or NWU

	cur_odom.pose.pose.orientation = att_q;
	odom_pub.publish(cur_odom);

	float vx = x/dt_us*1e3f;
	float vy = y/dt_us*1e3f;
}

void FlowReader::process_imu(const ros::MessageEvent<sensor_msgs::Imu const>& event)
{
	sensor_msgs::Imu::ConstPtr msg = event.getMessage();
	gyro_x = msg->angular_velocity.x;
	gyro_y = msg->angular_velocity.y;

	att_q = msg->orientation;

}

void FlowReader::process_range(const ros::MessageEvent<sensor_msgs::Range const>& event)
{
	sensor_msgs::Range::ConstPtr msg = event.getMessage();
	//sensor_msgs::Range range;
	altitude = msg->range;

	// We can use float d = agl() * cosf(euler.phi()) * cosf(euler.theta()); to correct if we choose to use sonar or baro
	
}


bool FlowReader::registerCallbacks(const ros::NodeHandle& n)
{
	ros::NodeHandle optical_node(n);
	images_sub = optical_node.subscribe("images", 1000, &FlowReader::process_images, this);
	imu_sub = optical_node.subscribe("imu", 3, &FlowReader::process_imu, this);
	range_sub = optical_node.subscribe("range", 3, &FlowReader::process_range, this);

	flow_pub = optical_node.advertise<geometry_msgs::TwistStamped>("flow", 1000, false);
	odom_pub = optical_node.advertise<nav_msgs::Odometry>("odom", 1000, false);
  return true;
}
