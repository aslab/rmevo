"""
Robot body generator based on the revolve code
"""
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
    Generalization from the revolve robot used in the RMEvo code
    """

    def __init__(self, _id=None, self_collide=True, self_factory=None):
        self._id = _id
        self._body = None
        self._brain = None
        self._morphological_measurements = None
        self._brain_measurements = None
        self._behavioural_measurements = None
        self.self_collide = self_collide
        self.battery_level = 0.0
        self.new_module = None
        self.factory = self_factory

    @property
    def id(self):
        return self._id

    @property
    def body(self):
        return self._body

    @property
    def brain(self):
        return self._brain

    def size(self):
        robot_size = 1 + self._recursive_size_measurement(self._body)
        return robot_size

    def _recursive_size_measurement(self, module):
        count = 0
        for _, child in module.iter_children():
            if child is not None:
                count += 1 + self._recursive_size_measurement(child)

        return count

    def measure_behaviour(self):
        """

        :return:
        """
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
        :return:
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
        Read robot's description from a file and parse it to Python structure
        :param path: Robot's description file path
        :param conf_type: Type of a robot's description format
        :return:
        """
        with open(path, 'r') as robot_file:
            robot = robot_file.read()

        self.load(robot, conf_type)

    def to_yaml(self):
        """
        Converts robot data structure to yaml

        :return:
        """
        yaml_dict = OrderedDict()
        yaml_dict['id'] = self._id
        yaml_dict['body'] = self._body.to_yaml()
        if self._brain is not None:
            yaml_dict['brain'] = self._brain.to_yaml()

        return yaml.dump(yaml_dict)

    def save_file(self, path, conf_type='yaml'):
        """
        Save robot's description on a given file path in a specified format
        :param path:
        :param conf_type:
        :return:
        """
        robot = ''
        if 'yaml' == conf_type:
            robot = self.to_yaml()

        with open(path, 'w') as robot_file:
            robot_file.write(robot)
