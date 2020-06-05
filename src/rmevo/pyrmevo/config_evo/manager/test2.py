#!/usr/bin/env python3
"""
This script is used for testing the SDF parser. It is used to simulated robots with modules imported from SDF files.
"""

import os
import sys
import asyncio
from pyrmevo.SDF.math import Vector3
from pyrmevo import rmevo_bot, parser
from pyrmevo.rmevo_bot.brain import BrainRLPowerSplines
from pyrmevo.tol.manage import World
from pyrmevo.util.supervisor.supervisor_multi import DynamicSimSupervisor
from pyrmevo.evolution import fitness

from pyrmevo.custom_logging.logger import logger

async def run():
    """
    The main coroutine, which is started below.
    """
    robot_file_path = "rmevo/test/centipede.yaml"
    #robot_file_path = "rmevo/test/snake.yaml"

    module_file_path = 'rmevo/test/module.sdf'
    module_file_dir = 'rmevo/test/modules'

    sdf_file_path = 'rmevo/test/robot.sdf'
    sdf_folder_path = 'rmevo/test'

    # Parse command line / file input arguments
    settings = parser.parse_args()

    settings.simulator_cmd = 'gazebo'

    # Start Simulator
    if settings.simulator_cmd != 'debug':
        simulator_supervisor = DynamicSimSupervisor(
            world_file=settings.world,
            simulator_cmd=settings.simulator_cmd,
            simulator_args=["--verbose"],
            plugins_dir_path=os.path.join('.', 'build', 'lib'),
            models_dir_path=os.path.join('.', 'models'),
            simulator_name='gazebo'
        )
        await simulator_supervisor.launch_simulator(port=settings.port_start)
        await asyncio.sleep(0.1)

    # Load modules from files
    logger.info("Starting Factory.")
    factory = rmevo_bot.Factory()
    logger.info("Importing module.")
    factory.import_modules_from_dir(module_file_dir)

    # Load a robot from yaml
    robot = rmevo_bot.RMEvoBot(self_factory=factory)
    logger.info("Loading Robot.")
    robot.load_file(robot_file_path)
    robot.update_substrate()
    #robot._brain = BrainRLPowerSplines()

    # robot._brain = BrainRLPowerSplines()

    # Print robot to sdf file
    # logger.info("Parsing robot to model.")
    # sdf_model = robot.to_sdf()
    # robot_sdf_file = open(sdf_file_path, 'w')
    # robot_sdf_file.write(sdf_model)
    # robot_sdf_file.close()

    # Connect to the simulator and pause
    connection = await World.create(settings, world_address=('127.0.0.1', settings.port_start))
    connection.output_directory = sdf_folder_path
    await asyncio.sleep(1)

    # Starts the simulation
    await connection.pause(False)

    # Insert the robot in the simulator
    robot_manager = await connection.insert_robot(robot, Vector3(0, 0, settings.z_start))

    # Start a run loop to do some stuff
    while True:
        # Print robot fitness every second
        status = 'dead' if robot_manager.dead else 'alive'
        print(f"Robot fitness ({status}) is: {fitness.displacement(robot_manager, robot)} \n")
        await asyncio.sleep(1.0)
