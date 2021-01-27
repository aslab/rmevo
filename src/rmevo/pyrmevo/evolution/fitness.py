import random as py_random

import rospy


class FitnessManager:
    """
    Fitness object that manages the fitness evaluation operation.
    Stores a link to the Gazebo instance used to evaluate the robots fitness.

    :param gazebo_manager:  link to the Gazebo instance used to evaluate the individuals.
    :type gazebo_manager: :class:`~pyrmevo.gazebo.gazebo_manager.GazeboManager`
    :param fitness_eval: function for fitness evaluation. Currently not in use.
    :param factory_manager: link to Factory used to generate and spawn the robots for evaluation.
    :type factory_manager: :class:`~pyrmevo.factory.factory_manager.FactoryManager`
    """
    def __init__(self,
                 gazebo_manager,
                 factory_manager,
                 fitness_eval=None):
        self.gazebo_manager = gazebo_manager
        self.fitness_eval = fitness_eval
        self.factory_manager = factory_manager

    def evaluate_fitness(self, individual):
        """
        Evaluates the fitness of the given individual.

        Resets the simulation and spawns the individual into the Gazebo World.
        Then calls the rmevo_gazebo service to evaluate the spawned robot.

        :param individual:  link to the Gazebo instance used to evaluate the individuals.
        :type individual: :class:`~pyrmevo.evolution.individual.Individual`
        """
        self.gazebo_manager.reset_simulation()
        robot_yaml = individual.genotype.to_yaml()
        self.factory_manager.send_robot_to_factory(individual.id, robot_yaml)
        rospy.sleep(2)
        fitness = self.gazebo_manager.evaluate_robot_fitness(individual.id)
        rospy.sleep(1)
        self.gazebo_manager.delete_robot(individual.id)
        return fitness


def stupid(_robot_manager, robot):
    return 1.0


def random(_robot_manager, robot):
    return py_random.random()


def displacement(robot_manager, robot):
    displacement_vec = measures.displacement(robot_manager)[0]
    displacement_vec.z = 0
    return displacement_vec.magnitude()


def displacement_velocity(robot_manager, robot):
    return measures.displacement_velocity(robot_manager)


def maximum_weight(robot_manager, robot):
    fitness = robot_manager.robot._morphological_measurements.width

    return fitness
