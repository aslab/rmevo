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
            # this split will give errors on windows
            manager_lib = os.path.splitext(arguments.manager)[0]
            manager_lib = '.'.join(manager_lib.split('/'))
            # manager = importlib.import_module(manager_lib,'pyrmevo.tutorials').run
            self.manager = importlib.import_module(manager_lib, package='rmevo').run

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
