from pyrmevo.genotype.plasticoding.plasticoding import Plasticoding
from pyrmevo.evolution.individual import Individual
from pyrmevo.custom_logging.logger import logger

import random
import copy

def generate_child_genotype(parent_genotypes, genotype_conf, crossover_conf):
    """
    Generates a child (individual) by randomly swaping two branches of the parents

    :param parents: parents to be used for crossover

    :return: child genotype
    """
    childs = []
    swaping_nodes = []
    crossover_attempt = random.uniform(0.0, 1.0)
    if crossover_attempt > crossover_conf.crossover_prob:
        childs.append(copy.deepcopy(parent_genotypes[0]))
        childs.append(copy.deepcopy(parent_genotypes[1]))
    else:
        childs.append(copy.deepcopy(parent_genotypes[0]))
        childs.append(copy.deepcopy(parent_genotypes[1]))
        # Node 0 is not included
        selected_node = random.randint(1, childs[0].size())
        swaping_nodes.append(childs[0].get_robot_module(selected_node))

        selected_node = random.randint(1, childs[1].size())
        swaping_nodes.append(childs[1].get_robot_module(selected_node))

        aux = swaping_nodes[0]
        swaping_nodes[0] = swaping_nodes[1]
        swaping_nodes[1] = aux

    return childs[0]


def standard_crossover(parent_individuals, genotype_conf, crossover_conf):
    """
    Creates an child (individual) through crossover with two parents

    :param parent_genotypes: genotypes of the parents to be used for crossover
    :return: genotype result of the crossover
    """
    parent_genotypes = [p.genotype for p in parent_individuals]
    new_genotype = generate_child_genotype(parent_genotypes, genotype_conf, crossover_conf)
    logger.info(
        f'crossover: for genome {new_genotype.id} - p1: {parent_genotypes[0].id} p2: {parent_genotypes[1].id}.')
    return new_genotype
