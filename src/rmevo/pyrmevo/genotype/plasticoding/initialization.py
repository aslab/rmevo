import random
import copy

from pyrmevo.rmevo_bot.rmevo_bot import RMEvoBot


def generate_random_children(conf):
    if random.random() > conf.empty_child_prob:
        new_module = copy.deepcopy(conf.factory.modules_list[random.randint(0, len(conf.factory.modules_list)-1)])

        for i in [0, len(new_module.children) - 1]:
            new_module.children[i] = generate_random_children(conf)

        return new_module

    else:
        return None


def generate_random_body(conf):
    modules_number = len(conf.factory.modules_list)
    module_template = conf.factory.modules_list[random.randint(0, modules_number-1)]
    new_body = copy.deepcopy(module_template)

    for i in enumerate(new_body.children):
        new_body.children[i] = generate_random_children(conf)

    return new_body


def rmevo_random_initialization(conf, next_robot_id):
    """
    Initializing a random genotype.
    :type conf: PlasticodingConfig
    :return: a Genome
    :rtype: Plasticoding
    """

    new_individual = RMEvoBot(_id=next_robot_id, self_factory=conf.factory)
    new_individual.set_body(generate_random_body(conf))

    return new_individual
