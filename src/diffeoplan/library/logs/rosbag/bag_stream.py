from . import LogItem, contract, read_bag, read_bag_state
from conf_tools.utils.expansion import expand_environment
import os
from .. import Stream

class BagStream(Stream):
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
            for y0, u, y1, x0 in read_bag(bagfile):
                yield LogItem(y0=y0, u=u, y1=y1, x0=x0)
                
    def read_all_state(self):
        print('read_all_state from bag_stream')
        """ Yields y0, u, y1 as numpy array over all files. """
        for bagfile in self.files:
            for y0, u, y1, x0 in read_bag_state(bagfile):
                yield LogItem(y0=y0, u=u, y1=y1, x0=x0)

