from diffeoplan.configuration.master import DiffeoplanConfig


def DDSFromSymbolic(resolution, actions, topology=None):

    for a in actions:
        diffeo = DiffeoplanConfig.symdiffeos.instance(a) #@UndefinedVariable
        
    pass
