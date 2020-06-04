#ifndef __GENERATOR_NODE_HPP__
#define __GENERATOR_NODE_HPP__

// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>

#include "std_msgs/Int8.h"
class NodeGenerator {
	public:
		//! Constructor.
		explicit NodeGenerator(int, char**);
		void Run();
		void OnGenerateMsg(const std_msgs::Int8);

	private:
		//! Subscriber to custom message.
		ros::Subscriber generation_sub_;
		int rate;
};

#endif  // __GENERATOR_NODE_HPP__
