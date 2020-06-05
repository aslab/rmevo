"""
Class containing the body parts to compose a RMEvo robot
"""
from collections import OrderedDict
from enum import Enum

from pyrmevo import SDF

import copy


# MEASUREMENT CONVERSION
def mm(x):
    return x / 1000.0


def cm(x):
    return x / 100.0


def grams(x):
    return x / 1000.0


# Module Orientation
class Orientation(Enum):
    SOUTH = 0
    NORTH = 1
    EAST = 2
    WEST = 3
    UP = 4
    DOWN = 5

    def short_repr(self):
        if self == self.SOUTH:
            return 'S'
        elif self == self.NORTH:
            return 'N'
        elif self == self.EAST:
            return 'E'
        elif self == self.WEST:
            return 'W'
        elif self == self.UP:
            return 'U'
        elif self == self.DOWN:
            return 'D'
        else:
            assert False


class RMEvoModule:
    """
    Base class allowing for constructing RMEvo components in an overviewable manner
    """
    DEFAULT_COLOR = (0.5, 0.5, 0.5)
    TYPE = None
    VISUAL_MESH = None
    COLLISION_BOX = None
    MASS = None
    INERTIA = None
    SDF = None

    def __init__(self):
        self.id = None
        self.orientation = None
        self.rgb = None  # RevolveModule.DEFAULT_COLOR
        self.substrate_coordinates = None
        self.children = [None, None, None, None]
        self.info = None
        self.robot = None

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

    def validate(self):
        """
        Tests if the robot tree is valid (recursively)
        :return: True if the robot tree is valid
        """
        raise RuntimeError("Robot tree validation not yet implemented")

    def to_sdf(self, tree_depth='', parent_link=None, child_link=None):
        """
        Transform the module in sdf elements.

        IMPORTANT: It does not append VISUAL and COLLISION elements to the parent link
        automatically. It does append automatically the SENSOR element.
        TODO: make the append automatic for VISUAL AND COLLISION AS WELL.

        :param tree_depth: current tree depth as string (for naming)
        :param parent_link: link of the parent (may be needed for certain modules)
        :param child_link: link of the child (may be needed for certain modules, like hinges)
        :return: visual SDF element, collision SDF element, sensor SDF element.
        Sensor SDF element may be None.
        """
        name = 'component_{}_{}__box'.format(tree_depth, self.TYPE)
        visual = SDF.Visual(name, self.rgb)
        geometry = SDF.MeshGeometry(self.VISUAL_MESH)
        visual.append(geometry)

        collision = SDF.Collision(name, self.MASS)
        geometry = SDF.BoxGeometry(self.COLLISION_BOX)
        collision.append(geometry)

        return visual, collision, None

    def boxslot(self, orientation=None):
        orientation = Orientation.SOUTH if orientation is None else orientation
        return BoxSlot(self.possible_slots(), orientation)

    def possible_slots(self):
        box_geometry = self.COLLISION_BOX
        return (
            (box_geometry[0] / -2.0, box_geometry[0] / 2.0),  # X
            (box_geometry[1] / -2.0, box_geometry[1] / 2.0),  # Y
            (box_geometry[2] / -2.0, box_geometry[2] / 2.0),  # Z
        )

    def has_children(self):
        """
        Check wheter module has children
        :return: True if module has children
        """
        has_children = False

        if self.children == {1: None}:
            return False

        for i, child in enumerate(self.children):
            if child is not None:
                has_children = True

        return has_children


class FactoryModule(RMEvoModule):

    """
    Inherits class RMEvoModule. Use module available from Factory
    """
    TYPE = None
    VISUAL_MESH = None
    COLLISION_BOX = None
    MASS = None
    SDF = None
    SDF_VISUAL = None
    SDF_COLLISION = None
    SDF_INERTIA = None
    SLOT_DATA = None

    def __init__(self):
        super().__init__()
        self.SLOT_DATA = []

    @staticmethod
    def FromYaml(yaml_object, factory):
        """
        From a yaml object, creates a data struture of interconnected body modules.
        Standard names for modules are:
        CoreComponent
        ActiveHinge
        FixedBrick
        FixedBrickSensor
        """

        new_module = FactoryModule()

        for module_template in factory.modules_list:
            if module_template.TYPE == yaml_object['type']:
                new_module = copy.deepcopy(module_template)
                break

        if new_module.TYPE is None:
            raise RuntimeError("Module not implemented")

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
                new_module.children[parent_slot] = FactoryModule.FromYaml(
                    yaml_object=yaml_object['children'][parent_slot], factory=factory)

        return new_module

    def to_sdf(self, tree_depth='', parent_link=None, child_link=None):
        from ..SDF.geometry import Material

        visual = self.SDF_VISUAL
        material = Material(
            ambient=(self.rgb[0], self.rgb[1], self.rgb[2], 1.0),
            diffuse=(self.rgb[0], self.rgb[1], self.rgb[2], 1.0),
            specular=(0.1, 0.1, 0.1, 1.0),
        )
        visual.append(material)

        collision = self.SDF_COLLISION

        if self.id == 'Core':
            imu_sensor = SDF.IMUSensor('core-imu_sensor', parent_link, self)
            parent_link.append(imu_sensor)
        else:
            imu_sensor = None

        return visual, collision, imu_sensor


class BoxSlot:
    """
    Helper class for modules connection slots
    """
    def __init__(self, boundaries, orientation: Orientation):
        self.orientation = orientation
        self.pos = self._calculate_box_slot_pos(boundaries, orientation)
        self.normal = self.pos.normalized()
        self.tangent = self._calculate_box_slot_tangent(orientation)

    def _calculate_box_slot_pos(self, boundaries, slot: Orientation):
        # boundaries = collision_elem.boundaries
        if slot == Orientation.SOUTH:
            return SDF.math.Vector3(0, boundaries[1][0], 0)
        elif slot == Orientation.NORTH:
            return SDF.math.Vector3(0, boundaries[1][1], 0)
        elif slot == Orientation.EAST:
            return SDF.math.Vector3(boundaries[0][1], 0, 0)
        elif slot == Orientation.WEST:
            return SDF.math.Vector3(boundaries[0][0], 0, 0)
        elif slot == Orientation.UP:
            return SDF.math.Vector3(0, 0, boundaries[2][0])
        elif slot == Orientation.DOWN:
            return SDF.math.Vector3(0, 0, boundaries[2][1])
        else:
            raise RuntimeError('invalid module orientation: {}'.format(slot))

    @staticmethod
    def _calculate_box_slot_tangent(slot: Orientation):
        """
        Return slot tangent
        """
        if slot == Orientation.SOUTH:
            return SDF.math.Vector3(0, 0, 1)
        elif slot == Orientation.NORTH:
            return SDF.math.Vector3(0, 0, 1)
        elif slot == Orientation.EAST:
            return SDF.math.Vector3(0, 0, 1)
        elif slot == Orientation.WEST:
            return SDF.math.Vector3(0, 0, 1)
        elif slot == Orientation.UP:
            return SDF.math.Vector3(0, 1, 0)
        elif slot == Orientation.DOWN:
            return SDF.math.Vector3(0, 1, 0)
        # elif slot == 4:
        #     # Right face tangent: back face
        #     return SDF.math.Vector3(0, 1, 0)
        # elif slot == 5:
        #     # Left face tangent: back face
        #     return SDF.math.Vector3(0, 1, 0)
        else:
            raise RuntimeError("Invalid orientation")


class BoxSlotJoints(BoxSlot):

    def __init__(self, boundaries, orientation: Orientation, offset=(SDF.math.Vector3(), SDF.math.Vector3())):
        self.offset = offset
        super().__init__(boundaries, orientation)

    def _calculate_box_slot_pos(self, boundaries, slot: Orientation):
        if slot == Orientation.SOUTH:
            return SDF.math.Vector3(boundaries[0][0], 0, 0) + self.offset[0]
        elif slot == Orientation.NORTH:
            return SDF.math.Vector3(boundaries[0][1], 0, 0) + self.offset[1]
        else:
            raise RuntimeError('invalid module orientation: {}'.format(slot))

    @staticmethod
    def _calculate_box_slot_tangent(slot: Orientation):
        """
        Return slot tangent
        """
        if slot == Orientation.SOUTH:
            return SDF.math.Vector3(0, 0, 1)
        elif slot == Orientation.NORTH:
            return SDF.math.Vector3(0, 0, 1)
        else:
            raise RuntimeError("Invalid orientation")


class BoxSlotTouchSensor(BoxSlot):
    def __init__(self, boundaries):
        super().__init__(boundaries, Orientation.SOUTH)

    def _calculate_box_slot_pos(self, boundaries, slot: Orientation):
        if slot == Orientation.SOUTH:
            return SDF.math.Vector3(boundaries[0][0], 0, 0)
        else:
            raise RuntimeError('invalid module orientation: {}'.format(slot))

    @staticmethod
    def _calculate_box_slot_tangent(slot: Orientation):
        """
        Return slot tangent
        """
        if slot == Orientation.SOUTH:
            return SDF.math.Vector3(0, 1, 0)
        else:
            raise RuntimeError("Invalid orientation")
