#!/usr/bin/env python3

import os
import asyncio
import sys
import importlib

import rospy

from pyrmevo.custom_logging.logger import logger
from pyrmevo.parser import parser


class RMEvoCore:
    def __init__(self, arguments):
        if arguments.manager is not None:
            if os.path.isfile(arguments.manager):
                spec = importlib.util.spec_from_file_location("manager", arguments.manager)
                manager_lib = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(manager_lib)
                self.manager = manager_lib.run
            else:
                raise ValueError("Given file does not exist.")

        else:
            raise ValueError("RMEvo needs manager argument")

    def run(self, loop, arguments):
        return loop.run_until_complete(self.manager())


class RMEvoNode:
    def __init__(self, loop, arguments):

        # Init Core in node
        rmevo_core = RMEvoCore(arguments)

        # Init node
        rospy.init_node('rmevo_core')

        # Start services
        #self.advertise_services()

        # Runs core
        rmevo_core.run(loop, arguments)

        # Keeps node spinning
        rospy.spin()

    # Registers all the services the node offers
    def advertise_services(self):
        raise NotImplementedError


def main():
    import traceback

    def handler(_loop, context):
        try:
            exc = context['exception']
        except KeyError:
            print(context['message'])
            return

        if isinstance(exc, ConnectionResetError):
            print("Got disconnect / connection reset - shutting down.")
            sys.exit(0)

        if isinstance(exc, OSError) and exc.errno == 9:
            print(exc)
            traceback.print_exc()
            return

        # traceback.print_exc()
        raise exc

    try:
        arguments = parser.parse_args()
        loop = asyncio.get_event_loop()
        loop.set_exception_handler(handler)
        RMEvoNode(loop, arguments)
    except KeyboardInterrupt:
        print("Got CtrlC, shutting down.")


if __name__ == '__main__':
    logger.info("Starting rmevo process")
    main()
