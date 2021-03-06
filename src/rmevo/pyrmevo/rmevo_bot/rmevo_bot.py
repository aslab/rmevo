import yaml
import traceback
from collections import OrderedDict
from collections import deque

from .rmevo_module import RMEvoModule

#from .measure.measure_body import MeasureBody

from ..custom_logging.logger import logger
import os


class RMEvoBot:
    """
    Robot body generator based on the revolve code.
    """

    def __init__(self, _id=None, self_collide=True, self_factory=None):
        self._id = _id
        self._body = None
        self.factory = self_factory

    @property
    def id(self):
        return self._id

    def set_id(self, new_id):
        self._id = new_id

    @property
    def body(self):
        return self._body

    def set_body(self, new_body):
        self._body = new_body

    def size(self):
        robot_size = 1 + self._recursive_size_measurement(self._body)
        return robot_size

    def _recursive_size_measurement(self, module):
        count = 0
        for _, child in module.iter_children():
            if child is not None:
                count += 1 + self._recursive_size_measurement(child)

        return count

    def _get_module(self, module, selected):
        """
        Returns the module with the number given in selected by a depth first count.

        There is probably another method more efficient
        """
        if selected == 0:
            return module
        else:
            cumulative_size = 0
            for _, child in module.iter_children():
                if child is not None:
                    next_size = self._recursive_size_measurement(child) + 1
                    if selected <= cumulative_size + next_size:
                        selected = selected - cumulative_size - 1
                        return self._get_module(child, selected)
                    else:
                        cumulative_size = cumulative_size + next_size

    def get_robot_module(self, selected):
        """
        Returns the module with the number given in selected by a depth first count.

        There is probably another method more efficient.
        """
        return self._get_module(self._body, selected)


    def measure_behaviour(self):
        pass

    def measure_phenotype(self):
        self._morphological_measurements = self.measure_body()
        self._brain_measurements = self.measure_brain()
        logger.info('Robot ' + str(self.id) + ' was measured.')

    # def measure_body(self):
    #     """
    #     :return: instance of MeasureBody after performing all measurements
    #     """
    #     if self._body is None:
    #         raise RuntimeError('Body not initialized')
    #     try:
    #         measure = MeasureBody(self._body)
    #         measure.measure_all()
    #         return measure
    #     except Exception as e:
    #         logger.exception('Failed measuring body')

    def export_phenotype_measurements(self, data_path):
        filepath = os.path.join(data_path, 'descriptors', f'phenotype_desc_{self.id}.txt')
        with open(filepath, 'w+') as file:
            for key, value in self._morphological_measurements.measurements_to_dict().items():
                file.write(f'{key} {value}\n')
            for key, value in self._brain_measurements.measurements_to_dict().items():
                file.write(f'{key} {value}\n')

    def load(self, text, conf_type):
        """
        Load robot's description from a string and parse it to Python structure

        :param text: Robot's description string
        :param conf_type: Type of a robot's description format
        """
        if 'yaml' == conf_type:
            self.load_yaml(text)
        elif 'sdf' == conf_type:
            raise NotImplementedError("Loading from SDF not yet implemented")

    def load_yaml(self, text):
        """
        Load robot's description from a yaml string

        :param text: Robot's yaml description
        """

        yaml_bot = yaml.safe_load(text)
        self._id = yaml_bot['id'] if 'id' in yaml_bot else None

        self._body = RMEvoModule.FromYaml(yaml_bot['body'], self.factory)

    def load_file(self, path, conf_type='yaml'):
        """
        Read robot's description from a file and parse it to Python structure.

        :param path: Robot's description file path
        :param conf_type: Type of a robot's description format
        """
        with open(path, 'r') as robot_file:
            robot = robot_file.read()

        self.load(robot, conf_type)

    def to_yaml(self):
        """
        Converts robot data structure to yaml.

        :return: yaml of the robot
        """
        yaml_dict = OrderedDict()
        yaml_dict['id'] = self._id
        yaml_dict['body'] = self._body.to_yaml()
        # if self._brain is not None:
        #     yaml_dict['brain'] = self._brain.to_yaml()

        return yaml.dump(yaml_dict)

    def save_file(self, path, conf_type='yaml'):
        """
        Save robot's description on a given file path in a specified format.

        :param path:
        :param conf_type:
        :return:
        """
        robot = ''
        if 'yaml' == conf_type:
            robot = self.to_yaml()

        with open(path, 'w') as robot_file:
            robot_file.write(robot)

    def regenerate_body_id(self):
        self._body.regenerate_id("")
