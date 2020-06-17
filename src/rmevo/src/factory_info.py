import rospy

from rmevo.srv import RobotConfiguration, OutputString

from pyrmevo.custom_logging.logger import logger
from pyrmevo.rmevo_bot.rmevo_module import RMEvoModule


class FactoryInfo:
    modules_list = None

    def __init__(self):
        self.modules_list = []

        # List of services
        self.generate_service = None
        self.list_modules_services = None

        self.get_factory_services()
        self.get_modules_list()

    def get_factory_services(self):
        # Try to find the service from the factory (has to be running)
        try:
            logger.info("Looking for factory services.")
            self.generate_service = rospy.ServiceProxy('/factory/generate_robot', RobotConfiguration)
            self.list_modules_services = rospy.ServiceProxy('factory/list_modules', OutputString)
        except:
            raise RuntimeError("Service not found, check if factory is running.")

    def get_modules_list(self):
        serv_res = self.list_modules_services()
        modules_string = serv_res.Output_message
        modules_string = modules_string.split("\n")
        for module in modules_string:
            if module is not '':
                new_module = RMEvoModule
                new_module.TYPE = module
                self.modules_list.append(new_module)

    def send_robot_to_factory(self, name, robot):
        logger.info("Spawning robot " + name)
        serv_res = self.generate_service(name, robot)
        if serv_res.success is True:
            logger.info("Robot spawned correctly")
        else:
            logger.error("Error spawning robot")
