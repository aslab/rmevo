#include <sdf/sdf.hh>
#include <ignition/math/Pose3.hh>
#include "gazebo/gazebo.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

#include <string.h>

// Include ros dependencies
#include <ros/ros.h>
#include <std_srvs/Empty.h>

// Project dependencies
#include <rmevo_gazebo/FitnessEvaluation.h>

namespace gazebo
{
  /**
     * \brief Class that controls the Gazebo World.
     * 
     * Child class of gazebo::WorldPlugin. Implements FitnessEvaluation method.
     * */
  class WorldControl : public WorldPlugin
  {
    private:
		physics::WorldPtr world;
		ros::NodeHandle nh_;
		ros::ServiceServer pause_simulation_service;
		ros::ServiceServer evaluate_fitness_service;

    public:
    /**
     * \brief Constructor.
     * */
    WorldControl() : nh_("worldcontrol") {
    }

    /**
     * \brief Starts the node and all its services,
     * 
     * Called at startup.
     * @param[in] world World element.
     * @param[in] sdf SDF of the World.
     * */
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
    }

    /**
     * \brief Advertise all the service of the node
     * 
     * Called at start up.
     * 
     * */
    void advertiseServices(){
        this->pause_simulation_service = this-> nh_.advertiseService("pause_simulation", &WorldControl::pauseSimulation, this);
        this->evaluate_fitness_service = this-> nh_.advertiseService("evaluate_fitness", &WorldControl::evaluateFitness, this);
    }

    /**
     * \brief Pauses the simulation.
     * 
     * This method is inherited from the Gazebo node parent, and is already available 
     * throught the proper service. 
     * 
     * This implementations serves as a test.
     * @param[in] req Service request (empty).
     * @param[out] res Service response (empty).
     * */
    bool pauseSimulation(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res){
        (void)req;
        (void)res;
        ROS_INFO_NAMED("WorldControl", "Pausing world...");
        world->SetPaused(true);
        return true;
    }

    /**
     * \brief Evaluates the fitness of the given robot
     * 
     * Callback function of the service `/world_control/evaluate_fitness`.
     * 
     * This function should be changed by the user to fit his needs. By default, it calls the function
     * \ref count_entity_children(gazebo::physics::BasePtr).
     * 
     * This implementations serves as a test.
     * @param[in] req Service request (empty).
     * @param[out] res Service response (empty).
     * */
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

        res.robot_fitness = count_entity_children(model);
        ROS_INFO_NAMED("WorldControl", "Fitness of the robot is %f.", res.robot_fitness);
        res.success = true;
        return true;
    }

    /**
     * \brief Example method of fitness function
     * 
     * Computes the number of children of the current link recursively.
     * 
     * This function serves as an example of fitness function. The user should write
     * a function that given an entity computes certain value related to some of its properties.
     *
     * @param[in] entity Parent entity whose children are counted.
     * \return Number of children of the entity.
     * */
    int count_entity_children(gazebo::physics::BasePtr entity){
        int all_children = 0;
        for (unsigned i = 0; i < entity->GetChildCount(); i++){
            all_children += 1;
            all_children += count_entity_children(entity->GetChild(i));
        }
        return all_children;
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(WorldControl)
}