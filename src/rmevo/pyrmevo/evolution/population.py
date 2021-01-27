# [(G,P), (G,P), (G,P), (G,P), (G,P)]

from pyrmevo.evolution.individual import Individual
from ..custom_logging.logger import logger
import time
import asyncio
import os


class PopulationConfig:
    """
    Object that sets the particular configuration for the population

    :param population_size: size of the population
    :param genotype_constructor: class of the genotype used
    :param genotype_conf: configuration for genotype constructor
    :param fitness_conf: configuration for the fitness evaluation
    :param mutation_operator: operator to be used in mutation
    :param mutation_conf: configuration for mutation operator
    :param crossover_operator: operator to be used in crossover
    :param selection: selection type
    :param parent_selection: selection type during parent selection
    :param population_management: type of population management ie. steady state or generational
    :param evaluation_time: duration of an experiment
    :param experiment_name: name for the folder of the current experiment
    :param experiment_management: object with methods for managing the current experiment
    :param offspring_size (optional): size of offspring (for steady state)
    """
    def __init__(self,
                 population_size: int,
                 genotype_constructor,
                 genotype_conf,
                 fitness_conf,
                 mutation_operator,
                 mutation_conf,
                 crossover_operator,
                 crossover_conf,
                 selection,
                 parent_selection,
                 population_management,
                 population_management_selector,
                 evaluation_time,
                 experiment_name,
                 experiment_management,
                 offspring_size=None,
                 next_robot_id=1):
        
        self.population_size = population_size
        self.genotype_constructor = genotype_constructor
        self.genotype_conf = genotype_conf
        self.fitness_manager = fitness_conf
        self.mutation_operator = mutation_operator
        self.mutation_conf = mutation_conf
        self.crossover_operator = crossover_operator
        self.crossover_conf = crossover_conf
        self.parent_selection = parent_selection
        self.selection = selection
        self.population_management = population_management
        self.population_management_selector = population_management_selector
        self.evaluation_time = evaluation_time
        self.experiment_name = experiment_name
        self.experiment_management = experiment_management
        self.offspring_size = offspring_size
        self.next_robot_id = next_robot_id


class Population:
    """
        Creates a Population object that initialises the
        individuals in the population with an empty list
        and stores the configuration of the system to the
        conf variable.

        :param conf: configuration of the system
        :param simulator_queue: connection to the simulator queue
        :param analyzer_queue: connection to the analyzer simulator queue
        :param next_robot_id: (sequential) id of the next individual to be created
        """
    def __init__(self, conf: PopulationConfig, simulator_queue, analyzer_queue=None, next_robot_id=1):
        self.conf = conf
        self.individuals = []
        self.analyzer_queue = analyzer_queue
        self.simulator_queue = simulator_queue
        self.next_robot_id = next_robot_id

    def _new_individual(self, genotype):
        individual = Individual(genotype)
        individual.genotype.regenerate_body_id()

        self.conf.experiment_management.export_genotype(individual)
        # self.conf.experiment_management.export_phenotype(individual)
        # self.conf.experiment_management.export_phenotype_images(os.path.join('data_fullevolution', 'phenotype_images'),
        #                                                         individual)

        return individual

    def init_pop(self, recovered_individuals=[]):
        """
        Populates the population (individuals list) with Individual objects that contains their respective genotype.
        
        It uses the genotype constructor defined in the configuration to build them.
        """
        for i in range(self.conf.population_size-len(recovered_individuals)):
            individual = self._new_individual(self.conf.genotype_constructor(self.conf.genotype_conf, self.next_robot_id))
            self.individuals.append(individual)
            self.next_robot_id += 1

        self.evaluate(self.individuals, 0)
        self.individuals = recovered_individuals + self.individuals

    def next_gen(self, gen_num, recovered_individuals=[]):
        """
        Creates next generation of the population through selection, mutation, crossover.

        :param gen_num: generation number
        :param individuals: recovered offspring
        :return: new population
        """

        new_individuals = []

        for _i in range(self.conf.offspring_size-len(recovered_individuals)):
            # Selection operator (based on fitness)
            # Crossover
            if self.conf.crossover_operator is not None:
                parents = self.conf.parent_selection(self.individuals)
                child_genotype = self.conf.crossover_operator(parents, self.conf.genotype_conf, self.conf.crossover_conf)
                child = Individual(child_genotype)
            else:
                child = self.conf.selection(self.individuals)

            child.genotype.set_id("Robot_" + str(self.next_robot_id))
            self.next_robot_id += 1

            # Mutation operator
            child_genotype = self.conf.mutation_operator(child.genotype, self.conf.mutation_conf)
            # Insert individual in new population
            individual = self._new_individual(child_genotype)

            new_individuals.append(individual)

        # evaluate new individuals
        self.evaluate(new_individuals, gen_num)

        new_individuals = recovered_individuals + new_individuals

        # create next population
        if self.conf.population_management_selector is not None:
            new_individuals = self.conf.population_management(self.individuals, new_individuals,
                                                              self.conf.population_management_selector)
        else:
            new_individuals = self.conf.population_management(self.individuals, new_individuals)
        new_population = Population(self.conf, self.simulator_queue, self.analyzer_queue, self.next_robot_id)
        new_population.individuals = new_individuals
        logger.info(f'Population selected in gen {gen_num} with {len(new_population.individuals)} individuals...')

        return new_population

    def evaluate(self, new_individuals, gen_num, type_simulation = 'evolve'):
        """
        Evaluates each individual in the new gen population.

        :param new_individuals: newly created population after an evolution iteration
        :param gen_num: generation number
        """
        robot_futures = []
        for individual in new_individuals:

            logger.info(f'Evaluation of Individual {individual.id}')
            individual.fitness = self.evaluate_single_robot(individual)

            logger.info(f'Individual {individual.id} has a fitness of {individual.fitness}')
            #TODO: Implement exporting

            if type_simulation == 'evolve':
                self.conf.experiment_management.export_fitness(individual)

    def evaluate_single_robot(self, individual):
        """
        :param individual: individual to evaluate
        :type individual: :class:`~pyrmevo.evolution.individual.Individual`
        :return: Returns fitness of individual
        """

        return self.conf.fitness_manager.evaluate_fitness(individual)

    def get_fittest_robot(self):
        """
        Returns the fittest robot of the population.

        :return: :class:`~pyrmevo.evolution.individual.Individual` with higher fitness
        """
        fittest_robot = self.individuals[0]
        for current_robot in self.individuals:
            if current_robot.fitness > fittest_robot.fitness:
                fittest_robot = current_robot

        return fittest_robot
