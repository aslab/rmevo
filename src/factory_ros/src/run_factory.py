#!/usr/bin/env python3

import sys

import rospy
from std_srvs.srv import Empty

from pyfactory.parser import parser
from pyfactory.factory import Factory
from pyfactory.custom_logging.logger import logger
from factory_ros.srv import ImportModules, OutputString


class FactoryNode:
    def __init__(self, arguments):
        if arguments.modules is None:
            logger.error("Must provide --modules argument with folder to modules")
            return

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
        self.factory.import_modules_from_dir(req)

    def get_modules_list(self, _req):
        modules_list = self.factory.get_modules_list()
        logger.info("Available modules are: ")
        for item in modules_list:
            logger.info("\t" + item.TYPE)
        return []

    def advertise_services(self):
        rospy.Service(rospy.get_name() + '/load_modules', ImportModules, self.import_modules_from_dir)
        rospy.Service(rospy.get_name() + '/list_modules', Empty, self.get_modules_list)


def main():
    arguments = parser.parse_args()
    try:
        FactoryNode(arguments)
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    logger.info("Starting factory process")
    main()
