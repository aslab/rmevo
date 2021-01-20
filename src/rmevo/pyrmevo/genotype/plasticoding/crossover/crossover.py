class CrossoverConfig:
    """
    Creates a Crossover object that sets the configuration for the crossover operator

    :param crossover_prob: crossover probability
    """
    def __init__(self,
                 crossover_prob):
        self.crossover_prob = crossover_prob
