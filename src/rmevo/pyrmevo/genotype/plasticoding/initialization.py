import random
import copy

from pyrmevo.rmevo_bot.rmevo_bot import RMEvoBot


def generate_random_children(conf, depth):
    if depth is 0:
        return None

    elif random.random() > conf.empty_child_prob:
        new_module = copy.deepcopy(conf.factory.modules_list[random.randint(0, len(conf.factory.modules_list)-1)])

        for i in range(len(new_module.children)):
            new_module.children[i] = generate_random_children(conf, depth-1)

        return new_module

    else:
        return None


def generate_random_body(conf):
    module_template = None
    for module in conf.factory.modules_list:
        if module.TYPE == "Core":
            module_template = module

    if module_template is not None:
        new_body = copy.deepcopy(module_template)
    else:
        raise RuntimeError("Module type Core not found")

    max_depth = conf.max_depth
    for i, children in enumerate(new_body.children):
        new_body.children[i] = generate_random_children(conf, max_depth)

    return new_body


def rmevo_random_initialization(conf, next_robot_id):
    """
    Initializing a random genotype.
    :type conf: PlasticodingConfig
    :return: a Genome
    :rtype: Plasticoding
    """
    robot_id = "Robot_0_" + str(next_robot_id)
    new_individual = RMEvoBot(_id=robot_id, self_factory=conf.factory)
    new_individual.set_body(generate_random_body(conf))

    return new_individual
