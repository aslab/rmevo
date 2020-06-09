#!/usr/bin/env python3

import rospy

from pyfactory.factory import Factory
import factory_ros.srv


class FactoryNode:
    def __init__(self):
        # Init node
        rospy.init_node('factory')

        # Runs factory in node
        self.factory = Factory()

        # Start services
        self.advertise_services()

        # Keeps node spinning
        rospy.spin()

    def import_modules_from_dir(self, req):
        self.factory.import_modules_from_dir(req)

    def advertise_services(self):
        rospy.Service('load_modules', factory_ros.srv.ImportModule, self.import_modules_from_dir)


if __name__ == '__main__':
    try:
        FactoryNode()
    except rospy.ROSInterruptException:
        pass
