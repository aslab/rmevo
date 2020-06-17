#!/usr/bin/env python3
"""
This script is an example of the integration with the factory node. It uses the factory to spawn the yaml model provided.
"""

import rospy


from factory_ros.srv import RobotConfiguration

from pyrmevo.custom_logging.logger import logger
from pyrmevo.parser import parser
from pyrmevo.rmevo_bot import rmevo_bot
from rmevo.src.factory_info import FactoryInfo

async def run():
    """
    The main coroutine, which is started below.
    """

    # File with the yaml file
    robot_file_path = "../test/centipede.yaml"

    # Parse command line / file input arguments
    settings = parser.parse_args()

    # Load interface with factory: gets the factory services and the available modules
    factory_info = FactoryInfo()

    # Load the robot from yaml
    robot = rmevo_bot.RMEvoBot(self_factory=factory_info)
    logger.info("Loading Robot.")
    robot.load_file(robot_file_path)

    # Parse the robot back to yaml
    # This means the core has imported correctly the robot structure
    robot_yaml = robot.to_yaml()

    # Calls the service and passes the robot yaml
    factory_info.send_robot_to_factory('basic_test_robot', robot_yaml)
