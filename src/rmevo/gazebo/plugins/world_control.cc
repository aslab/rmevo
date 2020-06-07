#include <sdf/sdf.hh>
#include <ignition/math/Pose3.hh>
#include "gazebo/gazebo.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

// Include ros dependencies
#include <ros/ros.h>
#include <std_srvs/Empty.h>

/// \example examples/plugins/world_edit.cc
/// This example creates a WorldPlugin, initializes the Transport system by
/// creating a new Node, and publishes messages to alter gravity.
namespace gazebo
{
  class WorldControl : public WorldPlugin
  {
    private:
		physics::WorldPtr world;
		ros::NodeHandle nh_;
		ros::ServiceServer pause_simulation_service;

    public:
    WorldControl() : nh_("worldcontrol_handler") {
    }

    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
        // Make sure the ROS node for Gazebo has already been initialized
        if (!ros::isInitialized())
        {
          ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
            << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
          return;
        }

        this->world = _world;

        ROS_INFO("Launching world controller plugin.");

        advertiseServices();

//        // Create a new transport node
//        transport::NodePtr node(new transport::Node());
//
//        // Initialize the node with the world name
//        node->Init(_parent->Name());
//
//        // Create a publisher on the ~/physics topic
//        transport::PublisherPtr physicsPub =
//        node->Advertise<msgs::Physics>("~/physics");
//
//        msgs::Physics physicsMsg;
//        physicsMsg.set_type(msgs::Physics::ODE);
//
//        // Set the step time
//        physicsMsg.set_max_step_size(0.01);

        // Change gravity
        //msgs::Set(physicsMsg.mutable_gravity(),
        //    ignition::math::Vector3d(0.01, 0, 0.1));
        //physicsPub->Publish(physicsMsg);
    }

    void advertiseServices(){
        this->pause_simulation_service = this-> nh_.advertiseService("pauseSimulation", &WorldControl::pauseSimulation, this);
    }

    bool pauseSimulation(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res){
        ROS_INFO_NAMED("WorldControl", "Pausing world...");
        world->SetPaused(true);
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(WorldControl)
}