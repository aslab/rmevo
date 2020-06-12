import rospy

from factory_ros.srv import RobotConfiguration

generate_service = rospy.ServiceProxy('/factory/generate_robot', RobotConfiguration)

with open('basic.yaml', 'r') as robot_file:
    robot = robot_file.read()

generate_service('basic_test_robot', robot)
