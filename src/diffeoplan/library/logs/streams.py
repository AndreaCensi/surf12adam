from abc import abstractmethod, ABCMeta

class Stream(object):
    """ 
        Abstracts away the source of the data. 
        
        Implement your own if you want to use different log formats.
         
    """
    
    __metaclass__ = ABCMeta
        
    @abstractmethod        
    def read_all(self):
        """ Yields a LogItem sequence. """
    
