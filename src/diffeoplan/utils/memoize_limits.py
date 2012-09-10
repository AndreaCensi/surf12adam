from collections import deque
from compmake.utils import duration_human
from contracts import contract, describe_type
from decorator import decorator
from diffeoplan.utils import WithInternalLog
from reprep.utils import frozendict2
import time
from psutil._compat import defaultdict

def memstring(num_bytes):
    K = 1000.0
    if num_bytes < K:
        return '%dB' % num_bytes
    if num_bytes < K * K:
        return '%.1fKB' % (num_bytes / K)
    if num_bytes < K * K * K:
        return '%.1fMB' % (num_bytes / (K * K))
    
    MB = num_bytes * 1 / (1000 * 1000.0)
    return '%.1fMB' % MB
    
def timestring(sec):
    if sec < 1:
        return '%.3f sec' % sec
    else:
        return duration_human(sec)

class CacheResult(object):
    def __init__(self, value, clock, wall, size=None):
        self.value = value
        self.clock = clock
        self.wall = wall
        self.num_calls = 0
        if size is None:
            size = value.__sizeof__()
        self.size = size
    
    def get_size_bytes(self):
        return self.size
    
    def get_value(self):
        self.num_calls += 1
        return self.value
    
    def __repr__(self):
        return ('CacheResult(size=%s,clock=%s,wall=%s,value=%s)' % 
                (memstring(self.size), timestring(self.clock), timestring(self.wall),
                 describe_type(self.value)))

class MemoizeCache(WithInternalLog):
    
    def __init__(self, obj, max_size=None, max_mem_MB=None):
        super(MemoizeCache, self).__init__()
        self.obj = obj
        self.max_size = max_size
        self.set_max_memory_MB(max_mem_MB)
        
        self.cur_mem_bytes = 0
        self.ncalls = 0
        self.nhits = 0
        self.nmiss = 0
        self.nerased = 0
        self.saved_wall = 0.0 # Seconds of wall time saved
        self.saved_clock = 0.0 # Seconds of clock time saved
        self.erased_wall = 0.0 # Seconds of wall time of entries erased
        self.erased_clock = 0.0 # Seconds of clock time of entries erased
        self.erased_bytes = 0 # Size of erased objects
        
        self.clear()
        
    def clear(self):
        # key -> CacheResult
        self.cache = {}
        # list of keys recently accessed
        self.accessed = deque()
        self.ignore = {} 
        
    def set_max_memory_MB(self, max_mem_MB):
        if max_mem_MB is not None:
            self.max_mem_bytes = max_mem_MB * 1000 * 1000
        else:
            self.max_mem_bytes = None
        
    def call(self, f, *args, **kwargs):
        assert len(self.cache) <= len(self.accessed)
        self.ncalls += 1
        key = self.make_key(f, args, kwargs)
        if not key in self.cache:
            self.nmiss += 1
            # Compute it 
            cache_result = self.compute(f, args, kwargs)
            self.log_computed(key, cache_result)
            self.possibly_add_new_result(key, cache_result)
            # Mark access
            assert not key in self.ignore
            self.ignore[key] = 1
            #print('Adding %s' % str(key))
            return cache_result.get_value()
        else:
            self.nhits += 1
            cache_result = self.cache[key]
            self.log_access(key, cache_result)
            # Mark access
            self.ignore[key] += 1
            #print('Marking %s with %s ' % (key, self.ignore[key]))
            #print('len %s %s' % (len(self.accessed), x))
            self.accessed.appendleft(key)
            self.saved_wall += cache_result.wall
            self.saved_clock += cache_result.clock
            return cache_result.get_value()
    
    
    def possibly_add_new_result(self, key, cache_result):
        """ Possibly adds the newly computed cache_result to the cache. """
        # We add it to the cache, and if we are over the limits
        # we will delete something.
        self.log_added(key, cache_result)
        self.add_to_cache(key, cache_result)
        self.accessed.appendleft(key)
        self.trim()
        
    def trim(self):
        while self.over_limit():
            #self.log_over_limit(key, cache_result)
            # We need to delete something.
            # Let's delete the thing that was accessed more remotely
            # (we can make variations on this strategy).
            oldest = self.accessed.pop()
            
            if not oldest in self.ignore:
                # bad cache: TODO solve this
                self.ignore[oldest] = 1
            
            assert oldest in self.ignore
            
            self.ignore[oldest] -= 1
                         
            if self.ignore[oldest] > 0:
                continue 

            assert oldest in self.cache
             
            cache_result = self.cache[oldest]
            self.log_removing(oldest, cache_result)
            self.delete_from_cache(oldest)
            del self.ignore[oldest]
            
            self.nerased += 1
            self.erased_wall += cache_result.wall
            self.erased_clock += cache_result.clock
            self.erased_bytes += cache_result.get_size_bytes()
           
    def add_to_cache(self, key, cache_result):
        self.cur_mem_bytes += cache_result.get_size_bytes()
        self.cache[key] = cache_result
        
    def delete_from_cache(self, key):
        cache_result = self.cache[key]
        self.cur_mem_bytes -= cache_result.get_size_bytes()
        del self.cache[key]
  
     
#    def remove_large_objects(self, min_size_MB):
#        """ Removes large objects from the cache. """
#        for key in list(self.cache.keys()):
#            cache_result = self.cache[key]
#            if cache_result.get_size_bytes() > min_size_MB * 1000 * 1000:
#                del self.cache[key]
#                # FIXME This does not necessarily work...
#                self.ignore[key] += 1
        
    def over_limit(self):
        """ Returns true if the cache is over the limit. """
        if (self.max_size is not None) and (len(self.cache) > self.max_size):
            self.log_over_limit_num(len(self.cache), self.max_size)
            return True
        if self.max_mem_bytes is not None:
            current = self.current_cache_bytes()
            if current > self.max_mem_bytes:
                self.log_over_limit_mem(current, self.max_mem_bytes)
                return True
        return False

                 
    def current_cache_bytes(self):
        """ Estimates the size of the current cache in bytes. """
        return self.cur_mem_bytes
        #return sum([x.get_size_bytes() for x in self.cache.values()])
       
    @contract(returns=CacheResult)
    def compute(self, f, args, kwargs):
        t0 = time.time()
        c0 = time.clock()
        value = f(self.obj, *args, **kwargs)
        clock = time.clock() - c0
        wall = time.time() - t0
        result = CacheResult(value=value, clock=clock, wall=wall)
        return result 
    
    def make_key(self, f, args, kwargs):
        """ 
            Returns an hashable key.
        
            We don't want to pickle functions, so we 
            use their name. Might this have problems with decorators?
            TODO: investigate
        """
        # TODO: use resolving names, so that equivalent calls
        # have the same signature
        key = (f.__name__, args, frozendict2(kwargs))
        return key
    
    def get_signature(self, key):
        """ Returns a signature for a calling key. """
        fname, args, kwargs = key
        args_str = ",".join(str(x) for x in args)
        kwargs_str = ",".join('%s=%s' % (k, v) for k, v in kwargs.items())
        signature = '%s:%s.%s(%s,%s)' % (self.obj.__class__.__name__,
                                         id(self.obj),
                                         fname,
                                         args_str, kwargs_str)
        return signature
    
    def log_over_limit(self, key, cache_result):
        pass
    
    def log_removing(self, key, cache_result): #@UnusedVariable
        pass
        #s = self.get_signature(key)
        #self.info('Erasing %s' % s)
    
    def log_added(self, key, cache_result): #@UnusedVariable
        pass
        #s = self.get_signature(key)
        #self.info('Adding to cache: %s' % s)
    
    def log_computed(self, key, cache_result):
        pass

    def log_access(self, key, cache_result):
        pass

    def log_over_limit_num(self, current, max_size):
        #self.info('Over limit: %s > %s' % (current, max_size))
        pass
    
    def log_over_limit_mem(self, current, max_bytes):
        #self.info('Over limit: %s > %s' % (memstring(current),
        #                                   memstring(max_bytes)))
        pass

    def summary(self):
        """ Returns a string summarizing the state of the cache. """
        def perc(a, tot):
            return '%.1f%%' % (100.0 * a / tot)
        
        s = "--- Cache for %s ---\n" % self.obj
        s += ' objects: %d  (limit: %s)\n' % (len(self.cache), self.max_size)
        s += '  memory: %s  (limit: %s)\n' % (memstring(self.current_cache_bytes()),
                                              memstring(self.max_mem_bytes) if
                                              self.max_mem_bytes else None)
        s += '  ncalls: %6d\n' % self.ncalls
        s += '   nhits: %6d (%s)\n' % (self.nhits, perc(self.nhits, self.ncalls))
        s += '   nmiss: %6d (%s)\n' % (self.nmiss, perc(self.nmiss, self.ncalls))
        s += '   len queue: %6d \n' % (len(self.accessed))
        s += '  ___Saved: \n'
        s += '   clock: %s\n' % timestring(self.saved_clock)
        s += '    wall: %s\n' % timestring(self.saved_wall)
        s += '  ___Erased objects:\n'
        s += '       nerased: %6d\n' % self.nerased
        s += '          wall: %s\n' % timestring(self.erased_wall)
        s += '         clock: %s\n' % timestring(self.erased_clock)
        s += '         bytes: %s\n' % memstring(self.erased_bytes)
        
        assert len(self.cache) <= len(self.accessed)
        n = 3
        n = min(n, len(self.cache) / 2)
        s += '\n'
        s += 'Last %d objects added to cache: \n' % n
        for i in range(n):
            key = self.accessed[i]
            s += '%4d) %s \n' % (i + 1, self.get_signature(key))
            s += '      -> %s\n' % (self.cache[key])
        s += '\n'
        s += 'Oldest %d objects added to cache: \n' % n
        size = len(self.cache)
        for i in range(size - n, size):
            key = self.accessed[i]
            s += '%4d) %s \n' % (i + 2, self.get_signature(key))
            s += '      -> %s\n' % (self.cache[key])
            
        return s
        

def memoize_limited(max_size=None, max_mem_MB=None):
    """ 
        Assumes the function is a function of a class.
        Puts the cache in the instance so that it can be pickled.
        
        Note that you have to be consistent with kwargs:
        
            def f(a,b,c):
                pass
                
            f(1,2,3)
            f(1,2,c=3) # different call
            
        It is assumed that the arguments to the functions are not big
        so that they can be all memorized.
    """        
    def memoize_impl_params(f2):
        def memoizer(f, obj, *args, **kwargs):
            if not 'memoize_cache' in obj.__dict__:
                obj.memoize_cache = MemoizeCache(obj,
                                                 max_size=max_size,
                                                 max_mem_MB=max_mem_MB)
            cache = obj.memoize_cache
            return cache.call(f, *args, **kwargs)             
        
        return decorator(memoizer, f2)
        
    return memoize_impl_params

    
