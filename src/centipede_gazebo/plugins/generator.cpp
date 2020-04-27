// Include Gazebo dependencies
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
//#include <gazebo_plugins/gazebo_ros_utils.h>

// Include ros dependencies
#include <ros/ros.h>
#include <std_srvs/Empty.h>

namespace gazebo
{	
	class Generator : public WorldPlugin
	{
		private:
		physics::WorldPtr world;
		ros::NodeHandle nh_;
		ros::ServiceServer busSpawner;
		ros::ServiceServer generatorService;
		
		public : 
		// Constructor
		Generator() :
			nh_("generator_node")
		{
		}

		// Destructor
		~Generator()
		{
		}
  
		void Load(physics::WorldPtr _world, sdf::ElementPtr /*_sdf*/) {
			// Make sure the ROS node for Gazebo has already been initialized                                                                                    
			if (!ros::isInitialized())
			{
			  ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
				<< "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
			  return;
			}
			
			this->world = _world;
        
			//gazebo_ros_->getParameter<std::string> ( command_topic_, "commandTopic", "cmd_vel" );
			//Get the joint
			//selected_joint_ = gazebo_ros_->getJoint(this->parent, "jointName", "joint1");
	
			ROS_INFO("Launching generator_controller");
			
			//generatorService = ros::NodeHandle::advertiseService("spawn_two_legged", SpawnTwoLegged);
			this->busSpawner = this->nh_.advertiseService("spawn_bus", &Generator::SpawnBus, this);
	
			//ros::SubscribeOptions so =
			//	ros::SubscribeOptions::create<sensor_msgs::JointState>(command_topic_, 1,
            //        boost::bind(&GazeboRosChangeMorphology::DetachJoint, this, _1),
            //        ros::VoidPtr(), &queue_);
			// First try to insert something simple
			//_parent->InsertModelFile("model://bus");
		}
				
		/*bool SpawnTwoLegged( ros::ServiceEvent<std_srvs::Empty, std_srvs::Empty>& event ){
			ROS_INFO_NAMED("Generator","Spawning twolegged... (Not writed yet)");
			return true;
		}
		*/
		bool SpawnBus( std_srvs::Empty::Request &req,std_srvs::Empty::Response &res){
			ROS_INFO_NAMED("Generator","Spawning bus...");
			world->InsertModelFile("model://bus");
			return true;
		}
	
	};
	
	// Register this plugin with the simulator
	GZ_REGISTER_WORLD_PLUGIN(Generator)
}
