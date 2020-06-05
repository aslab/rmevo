#!/usr/bin/env python3
"""
This script is used for testing the SDF parser. It is used to import modules from sdf files and use them to assemble robots.
It outputs the final robot to a SDF file.
"""

from ... import rmevo_bot, parser

from pyrmevo.custom_logging.logger import logger

factory = rmevo_bot.Factory()


async def run():
    """
    The main coroutine, which is started below.
    """
    #robot_file_path = "rmevo/test/basic.yaml"
    #robot_file_path = "rmevo/test/twomodules.yaml"
    robot_file_path = "test/basic_revolve.yaml"
    #robot_file_path = "experiments/examples/yaml/spider.yaml"
    module_file_path = 'test/modules/basic/module.sdf'
    sdf_file_path = 'test/robot.sdf'

    # Parse command line / file input arguments
    settings = parser.parse_args()

    # Load modules from files
    logger.info("Starting Factory.")
    logger.info("Importing module.")
    factory.import_module_from_sdf(module_file_path)

    # Load a robot from yaml
    robot = rmevo_bot.RMEvoBot(self_factory=factory)
    logger.info("Loading Robot.")
    robot.load_file(robot_file_path)
    robot.update_substrate()

    # Print robot to sdf file
    logger.info("Parsing robot to model.")
    sdf_model = robot.to_sdf()
    robot_sdf_file = open(sdf_file_path, 'w')
    robot_sdf_file.write(sdf_model)
    robot_sdf_file.close()
