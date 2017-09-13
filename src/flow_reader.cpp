#include <ros/ros.h>
#include <FlowReader.h>

int main (int argc, char **argv)
{
	ros::init(argc, argv, "flow_reader");
  	ros::NodeHandle n("~");

  	FlowReader px4flow;

  	if (!px4flow.initialize(n))
  {
    ROS_ERROR("%s: failed to initialize flow_reader", ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }

  ros::spin();

  return EXIT_SUCCESS;
}
