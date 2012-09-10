from . import SymbolicDiffeo, contract, np
from geometry import se2, SE2
import geometry

class MotionDiffeo(SymbolicDiffeo):
    """ A rigid motion of the plane/torus. """
    
    @contract(avel='number', lvel='seq[2](number)', topology='str',
              interval='>0')
    def __init__(self, lvel, avel, topology='plane', interval=1):
        '''
        :param lvel: Instantaneous linear velocity 
        :param avel: Instantaneous angular velocity
        :param topology: Topology of the domain (plane/torus)
        :param interval: Length of motion.
        '''
        
        self.topology_s = topology
        SymbolicDiffeo.__init__(self, topology)
        self.lvel = np.asarray(lvel)
        self.avel = float(avel)
        self.interval = interval
        
        self.v = se2.algebra_from_velocities(avel=self.avel, lvel=self.lvel)
        self.q = SE2.group_from_algebra(self.v)
        self.R, self.t = geometry.rotation_translation_from_SE2(self.q)
        
    def get_inverse(self):
        """ The inverse is just the motion with opposite velocity. """
        return MotionDiffeo(topology=self.topology_s,
                            lvel=(-self.lvel),
                            avel=(-self.avel),
                            interval=self.interval)
    
    @contract(point='array[2]', returns='array[2]')
    def apply(self, point):
        return np.dot(self.R, point) + self.t
        
    def __repr__(self):
        return ("MotionDiffeo(%s,%s,%s)" % 
                 (self.avel, self.lvel, self.topology_s))
