import random as py_random

import rospy


class FitnessManager:
    def __init__(self,
                 gazebo_manager,
                 factory_manager,
                 fitness_eval=None):
        """
        Creates Fitness object that manages the fitness evaluation operation

        :param gazebo_manager: class GazeboManager og the Gazebo instance
        :param fitness_eval: function for fitness evaluation
        :param factory_manager: class FactoryManager
        """
        self.gazebo_manager = gazebo_manager
        self.fitness_eval = fitness_eval
        self.factory_manager = factory_manager

    def evaluate_fitness(self, individual):
        self.gazebo_manager.reset_simulation()
        robot_yaml = individual.genotype.to_yaml()
        self.factory_manager.send_robot_to_factory(individual.id, robot_yaml)
        rospy.sleep(2)
        fitness = self.gazebo_manager.evaluate_robot_fitness(individual.id)
        self.gazebo_manager.delete_robot(individual.id)
        return fitness


def stupid(_robot_manager, robot):
    return 1.0


def random(_robot_manager, robot):
    return py_random.random()


#def gazebo_fitness(_robot_manager, robot):



def displacement(robot_manager, robot):
    displacement_vec = measures.displacement(robot_manager)[0]
    displacement_vec.z = 0
    return displacement_vec.magnitude()


def displacement_velocity(robot_manager, robot):
    return measures.displacement_velocity(robot_manager)


def maximum_weight(robot_manager, robot):
    fitness = robot_manager.robot._morphological_measurements.width

    return fitness
