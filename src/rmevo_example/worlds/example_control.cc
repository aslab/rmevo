#include <sdf/sdf.hh>
#include "gazebo/gazebo.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

#include <math.h>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <string.h>

// Include ros dependencies
#include <ros/ros.h>
#include <std_srvs/Empty.h>

// Project dependencies
#include <rmevo_gazebo/FitnessEvaluation.h>

namespace gazebo
{
  class WorldControl : public WorldPlugin
  {
    private:
		physics::WorldPtr world;
		ros::NodeHandle nh_;
		ros::ServiceServer pause_simulation_service;
		ros::ServiceServer evaluate_fitness_service;

		physics::LinkPtr target;

    public:
    WorldControl() : nh_("worldcontrol") {
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
        (void) _sdf;

        ROS_INFO("Launching world controller plugin.");

        advertiseServices();

        this->target = get_target_pointer(this->world);
    }

    void advertiseServices(){
        this->pause_simulation_service = this-> nh_.advertiseService("pause_simulation", &WorldControl::pauseSimulation, this);
        this->evaluate_fitness_service = this-> nh_.advertiseService("evaluate_fitness", &WorldControl::evaluateFitness, this);
    }

    bool pauseSimulation(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res){
        (void)req;
        (void)res;
        ROS_INFO_NAMED("WorldControl", "Pausing world...");
        world->SetPaused(true);
        return true;
    }

    bool evaluateFitness(rmevo_gazebo::FitnessEvaluation::Request &req, rmevo_gazebo::FitnessEvaluation::Response &res){
        ROS_INFO_NAMED("WorldControl", "Evaluating fitness of robot: %s", req.robot_id.c_str());
        // Currently a simple evaluation
        gazebo::physics::ModelPtr model = world->ModelByName(req.robot_id);

        if (!model)
        {
            ROS_ERROR_NAMED("WorldControl", "EvaluateRobot: model [%s] does not exist",req.robot_id.c_str());
            res.success = false;
            res.status_message = "EvaluateRobot: robot does not exist";
            return true;
        }

        res.robot_fitness = distance_to_target(model);
        ROS_INFO_NAMED("WorldControl", "Fitness of the robot is %f.", res.robot_fitness);
        res.success = true;
        return true;
    }

    int count_entity_children(gazebo::physics::BasePtr entity){
        int all_children = 0;
        for (unsigned i = 0; i < entity->GetChildCount(); i++){
            all_children += 1;
            all_children += count_entity_children(entity->GetChild(i));
        }
        return all_children;
    }

    gazebo::physics::LinkPtr get_target_pointer(physics::WorldPtr world){
        physics::ModelPtr rock = world->ModelByName("rock");
        physics::LinkPtr mineral = rock->GetChildLink("mineral");
        if (mineral != NULL)
          ROS_INFO_NAMED("WorldControl", "Obtained target pointer.");
        else
          ROS_ERROR_NAMED("WorldControl", "Didn't find target");
        return mineral;
    }

    float distance_to_target(gazebo::physics::ModelPtr entity){
        ignition::math::Vector3d target_position = this->target->WorldPose().Pos();
        ROS_INFO_NAMED("WorldControl", "Obtained target position.");
        for (unsigned int i = 0; i < entity->GetChildCount(); i++){
          char child_name = entity->GetChild(i)->GetName();
          int len = strlen(child_name);
          if ( strcmp(child_name[len-4], "tool"){
            physics::LinkPtr tool = entity->GetChild(i);
            break;
          }
        }
        if (tool == NULL){
          ROS_INFO_NAMED("WorldControl", "Tool not found");
          return 0;
        }
        ROS_INFO_NAMED("WorldControl", "Tool found.");
        ignition::math::Vector3d tool_position = tool->WorldPose().Pos();
        ROS_INFO_NAMED("WorldControl", "Tool position: %f", tool_position[1]);
        float distance = pow((target_position[0] - tool_position[0]), 2) + pow((target_position[1] - tool_position[1]), 2) + pow((target_position[2] - tool_position[2]), 2);
        return distance;
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(WorldControl)
}