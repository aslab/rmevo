import rospy, rosnode
from std_srvs.srv import Empty


class WorldLink:
    def __init__(self):
        if '/gazebo' not in rosnode.get_node_names():
            raise RuntimeError('Run gazebo first')
        self.reset_world_service = rospy.ServiceProxy('gazebo/reset_world', Empty)
        self.pause_world_physics = rospy.ServiceProxy('gazebo/pause_physics', Empty)

    def reset_world(self):
        try:
            self.reset_world_service()
        except rospy.service.ServiceException:
            raise RuntimeError('Cant reset the world')

    def pause_simulation(self):
        try:
            self.pause_world_physics()
        except rospy.service.ServiceException:
            raise RuntimeError('Cant pause the world')
