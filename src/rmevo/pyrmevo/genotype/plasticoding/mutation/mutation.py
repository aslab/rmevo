class MutationConfig:
    """
    Creates a MutationConfig object that sets the configuration for the mutation operator

    :param mutation_prob: mutation probability
    :param genotype_conf: configuration for the genotype to be mutated
    """
    def __init__(self,
                 mutation_prob,
                 genotype_conf):
        self.mutation_prob = mutation_prob
        self.genotype_conf = genotype_conf
