from abc import abstractmethod, ABCMeta

class Stream(object):
    """ A Stream is a set of log files which have the same configuration. """
    
    __metaclass__ = ABCMeta
        
    @abstractmethod        
    def read_all(self):
        """ Yields y0, u, y1 as numpy array over all files. """
    
