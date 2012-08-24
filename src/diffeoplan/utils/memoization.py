from reprep.report_utils.store_results import frozendict
from decorator import decorator


def memoize_simple(obj):
    cache = obj.cache = {}

    #@functools.wraps(obj)
    def memoizer(f, *args, **kwargs):
        key = (args, frozendict(kwargs))
        if key not in cache:
            cache[key] = f(*args, **kwargs)
        return cache[key]
    
    return decorator(memoizer, obj)
    
    #return memoizer


memoize_instance_show_store = False
memoize_instance_show_initial_cache = False
memoize_instance_stats_interval = 100000000

def memoize_instance(f2):
    """ 
        Assumes the function is a function of a class.
        Puts the cache in the instance so that it can be pickled.
        
        Note that you have to be consistent with kwargs:
        
            def f(a,b,c):
                pass
                
            f(1,2,3)
            f(1,2,c=3) # different call 
            
        
        
    """ 
    
    f2.__cache_calls = 0 
    f2.__cache_hits = 0
    
    #@functools.wraps(f)
    
    def memoizer(f, obj, *args, **kwargs):
        # just make sure everything clicks together
        has_this_method = f.__name__ in obj.__class__.__dict__
        if not has_this_method:
            msg = 'Class %s does not have method %s' % (obj.__class__, f.__name__)
            raise ValueError(msg)
        #method = obj.__class__.__dict__[f.__name__]
#        if memoizer != method:
#            msg = 'I expected %s = %s' % (method, memoizer)
#            raise ValueError(msg)
        
#        print('object %s' % (obj))
#        print('found: %s' % obj.__class__.__dict__[f.__name__])
#        print('should be %s' % memoizer)
#        print('memizing %s %s' % (f, type(f)))
#        if not isinstance(memoizer, MethodType):
#            msg = 'Warning, this will not work: '
#            msg += 'memoize_instance of %s %s ' % (memoizer, type(memoizer))
#            msg += 'not a MethodType.'
#            raise ValueError(msg)
        
        if not '__cache' in obj.__dict__:
            obj.__dict__['__cache'] = {}
        
        if not f.__name__ in obj.__cache:
            obj.__cache[f.__name__] = {}

        cache = obj.__cache[f.__name__]            
        if memoize_instance_show_initial_cache:
            if f.__cache_calls == 0 and len(cache) > 0: 
                # first time we call it
                print('For %r I already have a cache of %d calls' % 
                      (f.__name__, len(cache)))
                for key in cache:
                    print('- %s' % str(key))
        
        def get_signature():
            args_str = ",".join(str(x) for x in args)
            kwargs_str = ",".join('%s=%s' % (k, v) for k, v in kwargs.items())
            signature = '%s:%s.%s(%s,%s)' % (obj.__class__.__name__, id(obj), f.__name__,
                                             args_str, kwargs_str)
            return signature
        
        key = (args, frozendict(kwargs)) 
        if key not in cache:
            result = f(obj, *args, **kwargs)
            cache[key] = result
            if memoize_instance_show_store:
                print('STORE %s = %s' % (cache[key], get_signature()))
        else:
            f.__cache_hits += 1
            #print('LOAD  %s = %s' % (cache[key], signature))
            pass
        f.__cache_calls += 1         
        if f.__cache_calls % memoize_instance_stats_interval == 0:
            perc = 100.0 * f.__cache_hits / f.__cache_calls
            print('cache: %7d calls hits %3.3f%% for %s' % (f.__cache_calls,
                                                           perc, f.__name__)) 
        
        return cache[key]
    
    return decorator(memoizer, f2)
    #return memoizer


if __name__ == '__main__':
    class A():
        @memoize_instance
        def func1(self, a, b):
            return a + b
    
    a = A()
    a.func1(1, 2)
    a.func1(1, 2)

