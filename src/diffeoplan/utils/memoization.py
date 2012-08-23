import functools
from reprep.report_utils.store_results import frozendict


def memoize_simple(obj):
    cache = obj.cache = {}

    @functools.wraps(obj)
    def memoizer(*args):
        if args not in cache:
            cache[args] = obj(*args)
        return cache[args]
    return memoizer


def memoize_instance(f):
    """ 
        Assumes the function is a function of a class.
        Puts the cache in the instance so that it can be pickled.
    """
    @functools.wraps(f)
    def memoizer(obj, *args, **kwargs):
        if not '__cache' in obj.__dict__:
            obj.__dict__['__cache'] = {}
        cache = obj.__cache
         
        key = (obj, args, frozendict(kwargs)) 
        if key not in cache:
            result = f(obj, *args, **kwargs)
            cache[key] = result
        return cache[key]
    return memoizer


if __name__ == '__main__':
    class A():
        @memoize_instance
        def func1(self, a, b):
            return a + b
    
    a = A()
    a.func1(1, 2)
    a.func1(1, 2)

