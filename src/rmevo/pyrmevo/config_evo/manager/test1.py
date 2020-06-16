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
    #robot_file_path = "rmevo/test/basic.yaml"
    #robot_file_path = "rmevo/test/twomodules.yaml"
    robot_file_path = "../test/basic_revolve.yaml"
    #robot_file_path = "experiments/examples/yaml/spider.yaml"

    # Parse command line / file input arguments
    settings = parser.parse_args()

    # Init factory
    factory_info = FactoryInfo()



    # Load a robot from yaml
    robot = rmevo_bot.RMEvoBot(self_factory=None)
    logger.info("Loading Robot.")
    robot.load_file(robot_file_path)
    robot.update_substrate()

    # Calls the service and pases the robot yaml
    generate_service('basic_test_robot', robot)
