import pyrmevo.rmevo_bot.brain


class Brain(object):

    @staticmethod
    def from_yaml(yaml_brain):
        brain_type = yaml_brain['type']

        if brain_type == pyrmevo.rmevo_bot.brain.BrainNN.TYPE:
            return pyrmevo.rmevo_bot.brain.BrainNN.from_yaml(yaml_brain)
        elif brain_type == pyrmevo.rmevo_bot.brain.BrainRLPowerSplines.TYPE:
            return pyrmevo.rmevo_bot.brain.BrainRLPowerSplines.from_yaml(yaml_brain)
        elif brain_type == pyrmevo.rmevo_bot.brain.BrainCPGBO.TYPE:
            return pyrmevo.rmevo_bot.brain.BrainCPGBO.from_yaml(yaml_brain)
        else:
            return Brain()

    def to_yaml(self):
        return {}

    def learner_sdf(self):
        return None

    def controller_sdf(self):
        return None
