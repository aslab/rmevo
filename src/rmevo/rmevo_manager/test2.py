#!/usr/bin/env python3

"""
This file is a test that implements evolution using the available modules from the factory
"""


from pyrmevo.factory.factory_manager import FactoryManager
from pyrmevo.gazebo.gazebo_manager import GazeboManager

from pyrmevo.parser import parser
from pyrmevo.custom_logging.logger import logger

from pyrmevo.evolution.population import Population, PopulationConfig

from pyrmevo.genotype.plasticoding.initialization import rmevo_random_initialization

from pyrmevo.genotype.plasticoding.crossover.crossover import CrossoverConfig
from pyrmevo.genotype.plasticoding.crossover import rmevo_crossovers
# from pyrmevo.genotype.plasticoding.initialization import random_initialization
#
from pyrmevo.genotype.plasticoding.mutation.mutation import MutationConfig
from pyrmevo.genotype.plasticoding.mutation.rmevo_mutations import standard_mutation, null_mutation
from pyrmevo.genotype.plasticoding.plasticoding import PlasticodingConfig

from pyrmevo.evolution import fitness
from pyrmevo.evolution.selection import multiple_selection, tournament_selection
#
from pyrmevo.evolution.pop_management.steady_state import steady_state_population_management
from pyrmevo.experiment_management import ExperimentManagement
# from pyrmevo.util.supervisor.analyzer_queue import AnalyzerQueue
# from pyrmevo.util.supervisor.simulator_queue import SimulatorQueue
#
# from pyrmevo import rmevo_bot


async def run():
    """
    The main coroutine, which is started below.
    """

    # Load interface with factory: gets the factory services and the available modules
    factory_manager = FactoryManager()

    # Load interface with gazebo: gets services
    gazebo_manager = GazeboManager()

    # experiment params #
    num_generations = 5
    population_size = 10
    offspring_size = 5

    genotype_conf = PlasticodingConfig(
        max_depth=2,
        factory=factory_manager,
        axiom_w='Core',
        empty_child_prob=0.5
    )

    mutation_conf = MutationConfig(
        mutation_prob=0.8,
        genotype_conf=genotype_conf,
    )

    crossover_conf = CrossoverConfig(
        crossover_prob=0.8,
    )

    fitness_conf = fitness.FitnessManager(
        gazebo_manager=gazebo_manager,
        factory_manager=factory_manager
    )
    # experiment params #

    # Parse command line / file input arguments
    settings = parser.parse_args()
    settings.experiment_name = "Test2"
    settings.evaluation_time = 1

    experiment_management = ExperimentManagement(settings)
    # do_recovery = settings.recovery_enabled and not experiment_management.experiment_is_new()

    # logger.info('Activated run '+settings.run+' of experiment '+settings.experiment_name)

    # if do_recovery:
    #     gen_num, has_offspring, next_robot_id = experiment_management.read_recovery_state(population_size, offspring_size)
    #
    #     if gen_num == num_generations-1:
    #         logger.info('Experiment is already complete.')
    #         return
    # else:
    gen_num = 0
    next_robot_id = 1

    population_conf = PopulationConfig(
        population_size=population_size,
        genotype_constructor=rmevo_random_initialization,
        #genotype_constructor=random_initialization,
        genotype_conf=genotype_conf,
        fitness_conf=fitness_conf,
        mutation_operator=null_mutation,
        mutation_conf=mutation_conf,
        crossover_operator=rmevo_crossovers.standard_crossover,
        # crossover_operator=None,
        crossover_conf=crossover_conf,
        selection=lambda individuals: tournament_selection(individuals, 2),
        parent_selection=lambda individuals: multiple_selection(individuals, 2, tournament_selection),
        population_management=steady_state_population_management,
        population_management_selector=tournament_selection,
        evaluation_time=settings.evaluation_time,
        offspring_size=offspring_size,
        experiment_name=settings.experiment_name,
        experiment_management=experiment_management,
    )

    settings.output_directory = 'file'
    settings.simulator_cmd = 'gazebo'
    settings.world = 'worlds/plane.realtime.world'

    population = Population(population_conf, next_robot_id)

    # starting a new experiment
    experiment_management.create_exp_folders()
    population.init_pop()

    while gen_num < num_generations-1:
        gen_num += 1
        population = population.next_gen(gen_num)

    # output result after completing all generations...
    fittest_robot = population.get_fittest_robot()
    logger.info("Evolution finished")
    logger.info("Fittest robot is " + fittest_robot.id + " with a fitness of " + str(fittest_robot.fitness))
    logger.info("Process finished")
