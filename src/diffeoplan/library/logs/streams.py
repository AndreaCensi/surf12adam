from . import contract, np, logger
import os
from conf_tools.utils.expansion import expand_environment
from learner.programs.diffeo_learner.bag_reader import read_bag

class Stream:
    """ A Stream is a set of log files which have the same configuration. """
    
    @contract(files='list(str)')
    def __init__(self, files):
        files = map(expand_environment, files)
        for f in files:
            if not os.path.exists(f):
                msg = 'File %r does not exist.' % f 
                raise ValueError(msg)
        self.files = files
            
    def read_all(self):
        """ Yields y0, u, y1 as numpy array over all files. """
        for bagfile in self.files:
            for y0, u, y1 in read_bag(bagfile):
                yield y0, u, y1
