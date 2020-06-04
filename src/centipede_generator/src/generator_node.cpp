#include <centipede_generator/generator_node.hpp>
#include "std_msgs/Int8.h"

// Public
/// \brief Generator node: spawns models using the basic modules.
NodeGenerator::NodeGenerator(int argc, char ** argv){
	// Forward command line arguments to ROS
	//ros::init(argc, argv, "generator");
	//ros::NodeHandle n;

	// Subscribe to module messages
	//generation_sub_ = n.subscribe("generate_centipede", 2, &NodeGenerator::OnGenerateMsg, this);
	

	/// TODO: Change publisher to generate plugin
	/// cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", default_qos);
}

void NodeGenerator::Run() {
	// Tell ROS how fast to run this node.
	ros::Rate r(rate);
	ros::spin();
}

// Private
/// \brief Callback for generate_centipede
void NodeGenerator::OnGenerateMsg(const std_msgs::Int8 msg)
{
	// Load sdf

	// Send it to the generate plugin topic
	//cmd_pub_->publish(std::move(cmd_msg));
}

int main(int argc, char ** argv) {
	
	// Forward command line arguments to ROS
	ros::init(argc, argv, "generator");
	ros::NodeHandle n;

	NodeGenerator *nodetest = new NodeGenerator(argc, argv);
	
	// Subscribe to module messages
	ros::Subscriber generation_sub_ = n.subscribe<std_msgs::Int8>("generate_centipede", 2, &NodeGenerator::OnGenerateMsg, nodetest);
	
	//NodeGenerator *node_generator = new NodeGenerator(argc, argv);

	// Create a subscriber.
	// Name the topic, message queue, callback function with class name, and object containing callback function.
	//ros::Subscriber sub_message = n.subscribe(topic.c_str(), 1000, &NodeGenerator::messageCallback, node_generator);

	//node_generator->Run();
	
  
	// Main loop.
	/*while (n.ok())
	{
		ros::spinOnce();
		r.sleep();
	}*/

	return 0;
}
