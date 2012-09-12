from diffeoplan.utils.memoize_limits import memoize_limited, WithMemoizeCache

class Memoized(WithMemoizeCache):
    def __init__(self):
        WithMemoizeCache.__init__(self, max_size=None, max_mem_MB=25)
    
    @staticmethod
    def dec():
        """ Returns the decorator """
        return memoize_limited(max_size=None, max_mem_MB=25)
    
    def get_cache_stats(self):
        return self.memoize_cache.get_stats()
    
    def clear_cache(self):
        self.memoize_cache.clear()
    
    
def dp_memoize_instance(f):
    """ 
        This is the decorator used for functions that return 
        large objects, such as UncertainImage and Diffeomorphism2D.
        
        This is a place where we can turn on/off the memoization.
        
        In the future we will have a smarter strategy.
    """
     
    memoize = memoize_limited(max_size=None, max_mem_MB=25)
    return memoize(f)
    
    
    
