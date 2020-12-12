import rospy

from rmevo.srv import RobotConfiguration, OutputString

from pyrmevo.custom_logging.logger import logger
from pyrmevo.rmevo_bot.rmevo_module import RMEvoModule


class InternalParam:
    def __init__(self, name, type, range):
        self.name = name
        self.type = type
        aux = range.split(' ')
        self.min = aux[0]
        self.max = aux[1]


class FactoryManager:
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
            self.list_internal_params = rospy.ServiceProxy('factory/module_internal_params', OutputString)
        except:
            raise RuntimeError("Service not found, check if factory is running.")

    def get_modules_list(self):
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

                internal_params_srv = self.list_internal_params(module_type)
                internal_params_string = internal_params_srv.Ouput_message.split('n')
                for param in internal_params_string:
                    param = param.split(' ')
                    new_param = InternalParam(param[0], param[1], param[2], param[3])
                    new_module.internal_params.append(new_param)

                self.modules_list.append(new_module)

    def send_robot_to_factory(self, name, robot):
        logger.info("Spawning robot " + name)
        serv_res = self.generate_service(name, robot)
        if serv_res.success is True:
            logger.info("Robot spawned correctly")
        else:
            logger.error("Error spawning robot")
