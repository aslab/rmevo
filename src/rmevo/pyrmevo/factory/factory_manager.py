import rospy

from rmevo.srv import RobotConfiguration, OutputString

from pyrmevo.custom_logging.logger import logger
from pyrmevo.rmevo_bot.rmevo_module import RMEvoModule


class FactoryManager:
    """
    Object class that manages the conection between the :ros:pkg:`rmevo`
    and :ros:pkg:`factory_ros`.

    Stores the list of modules that the :ros:pkg:`rmevo` uses as list of posible aleles and
    offers the methods needed to comunicate with the the factory node.
    """
    modules_list = None

    def __init__(self):
        self.modules_list = []

        # List of services
        self.generate_service = None
        self.list_modules_services = None

        self.get_factory_services()
        self.get_modules_list()

    def get_factory_services(self):
        """
        Find the services that the :ros:pkg:`factory_ros` advertises.
        
        Store the ROS proxies used to
        call this services later in the program.
        """
        # Try to find the service from the factory (has to be running)
        try:
            logger.info("Looking for factory services.")
            self.generate_service = rospy.ServiceProxy('/factory/generate_robot', RobotConfiguration)
            self.list_modules_services = rospy.ServiceProxy('factory/list_modules', OutputString)
        except:
            raise RuntimeError("Service not found, check if factory is running.")

    def get_modules_list(self):
        """
        Calls the service :func:`/factory_ros/list_modules()`
        using the proxy obtained by :meth:`get_factory_services()`, which asks the factory node
        for the modules that it has imported.

        The string obtained is parsed into a list of available modules, that is stored in
        `self.modules_list`.
        """
        serv_res = self.list_modules_services()
        modules_string = serv_res.Output_message
        modules_string = modules_string.split("\n")
        for module in modules_string:
            if module is not '':
                new_module = RMEvoModule()
                module = module.replace('[', '')
                module = module.replace(']', '')
                module_type, module_slots = module.split(', ')

                new_module.TYPE = module_type
                for _i in range(int(module_slots)):
                    new_module.children.append([None])

                self.modules_list.append(new_module)

    def send_robot_to_factory(self, name, robot):
        """
        Calls the service :func:`/factory_ros/generate_robot()`
        using the proxy obtained by :meth:`get_factory_services()`.

        Through this service this method sends a robot and its morphological configuration
        to the factory and commands it to spawn it in Gazebo.

        :param name: Name of the robot. Used as ID in the program.
        :type name: String
        :param robot: String with yaml format that contains the genotype of the robot,
                        represented as a tree structure with the modules as nodes.
        :type robot: String
        """
        logger.info("Spawning robot " + name)
        serv_res = self.generate_service(name, robot)
        if serv_res.success is True:
            logger.info("Robot spawned correctly")
        else:
            logger.error("Error spawning robot")
