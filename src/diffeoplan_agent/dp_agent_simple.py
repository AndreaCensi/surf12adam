from bootstrapping_olympics.interfaces.agent import (AgentInterface,
    ServoAgentInterface)
from conf_tools.code_specs import instantiate_spec
from contracts import contract
from diffeoplan.configuration.master import get_dp_config
from diffeoplan.library.discdds.diffeo_system import DiffeoSystem
from abc import abstractmethod


class DiffeoPlanAgent(AgentInterface):
    
    @staticmethod
    def from_yaml(discdds, servo):
        dp_config = get_dp_config()
        _, i = dp_config.discdds.instance_smarter(discdds)
        return DiffeoPlanAgent(i, servo)

    @contract(discdds=DiffeoSystem)
    def __init__(self, discdds, servo):
        self.discdds = discdds
        self.servo = servo
        
    def init(self, boot_spec):
        self.boot_spec = boot_spec

    def get_predictor(self):
        raise NotImplementedError()

    def get_servo(self):
        servo_agent = instantiate_spec(self.servo)
        servo_agent.init(self.boot_spec)
        assert isinstance(servo_agent, DiffeoServoAgentInterface)
        servo_agent.set_discdds(self.discdds)
        return servo_agent

    def process_observations(self, observations):
        '''
            Process new observations.
            
            :param observations: a structure of type Observations
        '''

    @contract(returns='array')
    def choose_commands(self):
        raise NotImplementedError()



class DiffeoServoAgentInterface(ServoAgentInterface):
    
    @abstractmethod
    def set_discdds(self, discdds):
        pass


    
    
    
    
