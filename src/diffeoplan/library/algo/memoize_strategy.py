from diffeoplan.utils.memoize_limits import memoize_limited


def dp_memoize_instance(f):
    """ 
        This is the decorator used for functions that return 
        large objects, such as UncertainImage and Diffeomorphism2D.
        
        This is a place where we can turn on/off the memoization.
        
        In the future we will have a smarter strategy.
    """
    
#    if True:
#        return memoize_instance(f)
#    
    memoize = memoize_limited(max_size=None, max_mem_MB=25)
    return memoize(f)
    
    
    
