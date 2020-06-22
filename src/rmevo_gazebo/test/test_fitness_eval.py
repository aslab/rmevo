import rospy

from gazebo_msgs.srv import SpawnModel
from rmevo_gazebo.srv import FitnessEvaluation

from geometry_msgs.msg import Pose

spawn_service = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
fitness_service = rospy.ServiceProxy('/worldcontrol/evaluate_fitness', FitnessEvaluation)

with open('centipede.sdf', 'r') as robot_file:
    robot = robot_file.read()

# Init pose
pose = Pose()
pose.position.x = 0.0
pose.position.y = 0.0
pose.position.z = 0.0

pose.orientation.x = 0.0
pose.orientation.y = 0.0
pose.orientation.z = 0.0

reference_frame = ''

# Spawn robot from file
spawn_service('centipede_robot', robot, 'centipede_robot', pose, reference_frame)

# Wait for robot to spawn
rospy.sleep(10)

# Eval fitness
fitness = fitness_service('centipede_robot')
print(fitness)
