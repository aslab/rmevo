# (G,P)


class Individual:
    """
    Object that represent each single individual.
    Stores his genotype and fitness.

    :param genotype: genotype of the individual.
    :type genotype: :class:`~pyrmevo.genotype.genotype.Genotype`
    :param phenotype (optional): not in use.
    """
    def __init__(self, genotype, phenotype=None):
        self.genotype = genotype
        self.fitness = None
        self.parents = None
        self.failed_eval_attempt_count = 0

    @property
    def id(self):
        _id = None
        _id = self.genotype.id
        return _id

    def export_genotype(self, folder):
        self.genotype.save_file(f'{folder}/genotypes/genotype_{self.id}.txt')

    def export_fitness(self, folder):
        """
        It's saving the fitness into a file. The fitness can be a floating point number or None
        :param folder: folder where to save the fitness
        """
        with open(f'{folder}/fitness_{self.id}.txt', 'w') as f:
            f.write(str(self.fitness))

    def export(self, folder):
        self.export_genotype(folder)
        self.export_fitness(folder)

    def __repr__(self):
        return f'Individual_{self.id}({self.fitness})'
