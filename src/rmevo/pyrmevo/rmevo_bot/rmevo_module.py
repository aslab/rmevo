"""
Class containing the body parts to compose a RMEvo robot
"""
from collections import OrderedDict
from enum import Enum

import copy


# MEASUREMENT CONVERSION
def mm(x):
    return x / 1000.0


def cm(x):
    return x / 100.0


def grams(x):
    return x / 1000.0


class RMEvoModule:
    """
    Base class allowing for constructing RMEvo components in an overviewable manner
    """
    DEFAULT_COLOR = (0.5, 0.5, 0.5)
    TYPE = None

    def __init__(self):
        self.id = None
        self.orientation = None
        self.rgb = None  # RevolveModule.DEFAULT_COLOR
        self.substrate_coordinates = None
        self.children = []
        self.info = None
        self.robot = None
        self.internal_params = []

    def color(self):
        return self.rgb if self.rgb is not None else self.DEFAULT_COLOR

    def to_yaml(self):
        if self.TYPE is None:
            raise RuntimeError('Module TYPE is not implemented for "{}",'
                               ' this should be defined.'.format(self.__class__))

        yaml_dict_object = OrderedDict()
        yaml_dict_object['id'] = self.id
        yaml_dict_object['type'] = self.TYPE
        yaml_dict_object['orientation'] = self.orientation

        if self.rgb is not None:
            yaml_dict_object['params'] = {
                'red': self.rgb[0],
                'green': self.rgb[1],
                'blue': self.rgb[2],
            }

        children = self._generate_yaml_children()
        if children is not None:
            yaml_dict_object['children'] = children

        return yaml_dict_object

    def iter_children(self):
        return enumerate(self.children)

    def _generate_yaml_children(self):
        has_children = False

        children = {}
        for i, child in self.iter_children():
            if child is not None:
                children[i] = child.to_yaml()
                has_children = True

        return children if has_children else None

    def has_children(self):
        """
        Check whether module has children
        :return: True if module has children
        """
        has_children = False

        if self.children == {1: None}: return False

        for i, child in enumerate(self.children):
            if child is not None:
                has_children = True

        return has_children

    @staticmethod
    def FromYaml(yaml_object, factory):
        """
        From a yaml object, creates a data struture of interconnected body modules.
        """

        new_module = RMEvoModule()

        for module_template in factory.modules_list:
            if module_template.TYPE == yaml_object['type']:
                new_module = copy.deepcopy(module_template)
                break

        if new_module.TYPE == None:
                assert RuntimeError("Module not implemented")

        new_module.id = yaml_object['id']

        try:
            new_module.orientation = yaml_object['orientation']
        except KeyError:
            new_module.orientation = 0

        try:
            new_module.rgb = (
                yaml_object['params']['red'],
                yaml_object['params']['green'],
                yaml_object['params']['blue'],
            )
        except KeyError:
            pass

        if 'children' in yaml_object:
            for parent_slot in yaml_object['children']:
                new_module.children[parent_slot] = RMEvoModule.FromYaml(
                    yaml_object=yaml_object['children'][parent_slot], factory=factory)

        return new_module

    def regenerate_id(self, chain_string):
        self.id = self.TYPE + chain_string
        for i, child in enumerate(self.children):
            if child is not None:
                child.regenerate_id(chain_string + "_" + str(i))
