# (G,P)


class Individual:
    def __init__(self, genotype, phenotype=None):
        """
        Creates an Individual object with the given genotype and optionally the phenotype.

        :param genotype: genotype of the individual
        :param phenotype (optional): phenotype of the individual
        """
        self.genotype = genotype
        self.fitness = None
        self.parents = None
        self.failed_eval_attempt_count = 0

    @property
    def id(self):
        _id = None
        if self.phenotype is not None:
            _id = self.phenotype.id
        elif self.genotype.id is not None:
            _id = self.genotype.id
        return _id

    def export_genotype(self, folder):
        self.genotype.export_genotype(f'{folder}/genotypes/genotype_{self.phenotype.id}.txt')

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
