import rospy


from std_srvs.srv import Empty
from gazebo_msgs.srv import DeleteModel

from rmevo.srv import FitnessEvaluation

from pyrmevo.custom_logging.logger import logger
from pyrmevo.rmevo_bot.rmevo_module import RMEvoModule


class GazeboManager:

    def __init__(self):
        # List of services
        self.reset_service = None
        self.fitness_service = None
        self.delete_model_service = None

        self.get_gazebo_services()

    def get_gazebo_services(self):
        # Try to find the services from gazebo (has to be running)
        try:
            logger.info("Looking for gazebo services.")
            self.reset_service = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
            self.fitness_service = rospy.ServiceProxy('/worldcontrol/evaluate_fitness', FitnessEvaluation)
            self.delete_model_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        except:
            raise RuntimeError("Service not found, check if factory is running.")

    def evaluate_robot_fitness(self, robot_name):
        logger.info("Asking for fitness of robot " + robot_name)
        serv_res = self.fitness_service(robot_name)
        if serv_res.success is True:
            logger.info("Fitness of robot " + robot_name + "is " + str(serv_res.robot_fitness))
            return serv_res.robot_fitness
        else:
            logger.error("An error occurred while asking for the fitness: " + serv_res.status_message)
            return 0

    def reset_simulation(self):
        serv_res = self.reset_service()

    def delete_robot(self, robot_name):
        serv_res = self.delete_model_service(robot_name)

        if serv_res.success is True:
            logger.info("Robot with name " + robot_name + "deleted sucessfully.")
            return True
        else:
            logger.error("An error occurred while deleting the model: " + serv_res.status_message)
            return 0
