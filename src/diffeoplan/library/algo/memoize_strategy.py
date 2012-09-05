from diffeoplan.utils.memoization import memoize_instance


def dp_memoize_instance(f):
    """ 
        This is the decorator used for functions that return 
        large objects, such as UncertainImage and Diffeomorphism2D.
        
        This is a place where we can turn on/off the memoization.
        
        In the future we will have a smarter strategy.
    """
    
    if True:
        return memoize_instance(f)
    else:
        return f
    
    
    
