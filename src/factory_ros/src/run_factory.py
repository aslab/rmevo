#!/usr/bin/env python3

import os
import shutil

import rospy
from std_srvs.srv import Empty

from gazebo_msgs.srv import SpawnModel

from pyfactory.parser import parser
from pyfactory.factory import Factory
from pyfactory.custom_logging.logger import logger
from factory_ros.srv import ImportModules, OutputString, RobotConfiguration


class FactoryNode:
    """ This class implements the Factory Node.

    The Factory Node loads the modules from the given directory. It implements the methods that
    combine those modules into complex configurations and spawn them into the Gazebo World.

    :param arguments: Arguments given in the `rosrun` executable.
    """
    def __init__(self, arguments):
        """ Constructor method. Initializes and starts the node
        """
        self.arguments = arguments
        if arguments.modules is None:
            logger.error("Must provide --modules argument with folder to modules")
            return

        if arguments.output_folder is not None:
            if os.path.exists(arguments.output_folder):
                shutil.rmtree(arguments.output_folder)
            os.makedirs(arguments.output_folder)
            os.mkdir(os.path.join(arguments.output_folder, 'sdf'))

        # Init node
        rospy.init_node('factory')

        # Runs factory in node
        self.factory = Factory()
        self.factory.import_modules_from_dir(arguments.modules)

        # Start services
        self.advertise_services()

        # Keeps node spinning
        rospy.spin()

    def import_modules_from_dir(self, req):
        """Service callback that uses the factory method import_modules_from_dir to import the modules
        from the given folder.
            
        :param req: :ros:srv:`~factory_ros/ImportModules` `request`.
        """
        self.factory.import_modules_from_dir(req)

    def get_modules_list(self, req):
        """ Service callback that gets the modules list from the factory class and returns it as a string.

        :param req: :ros:srv:`~factory_ros/OutputString` `request`.
        :return: Modules list.
        """
        modules_list = self.factory.get_modules_list()
        modules_string = ""

        logger.info("Available modules are: ")
        for module in modules_list:
            next_module = "[" + module.TYPE + ", " + str(len(module.children)) + "]"
            logger.info("\t" + next_module)

            modules_string = modules_string + next_module + "\n"
        return modules_string

    def set_pose(self, position=[0, 0, 0], orientation=[0, 0, 0]):
        """ Generates a pose object with the information given in the parameters.

        :param position: Position for the Pose object, defaults to [0, 0, 0].
        :type position: list, optional
        :param orientation: Orientation for the Pose object, defaults to [0, 0, 0].
        :type orientation: list, optional
        """
        from geometry_msgs.msg import Pose
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]

        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]

        return pose

    def generate_robot(self, req):
        """ Generates the sdf for the configuration given and spawns it into gazebo.

        :param req: :ros:srv:`~factory_ros/RobotConfiguration` request.
        """
        model_name = req.model_name
        yaml_string = req.model_yaml
        robot_namespace = model_name
        initial_pose = self.set_pose([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        reference_frame = ""
        model_xml = self.factory.generate_sdf(model_name, yaml_string)

        if self.arguments.output_folder is not None:
            with open(f'{self.arguments.output_folder}/sdf/{model_name}.sdf', 'w') as robot_file:
                robot_file.write(model_xml)

        spawn_model_gazebo = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn_model_gazebo(model_name, model_xml, robot_namespace, initial_pose, reference_frame)

        return [True, '']

    def advertise_services(self):
        """ Starts all the services the node offers.
        """
        rospy.Service(rospy.get_name() + '/load_modules', ImportModules, self.import_modules_from_dir)
        rospy.Service(rospy.get_name() + '/list_modules', OutputString, self.get_modules_list)
        rospy.Service(rospy.get_name() + '/generate_robot', RobotConfiguration, self.generate_robot)


def main():
    arguments = parser.parse_args()
    try:
        FactoryNode(arguments)
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    logger.info("Starting factory process")
    main()
