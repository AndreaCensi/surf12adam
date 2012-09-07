from diffeoplan.utils.memoize_limits import memoize_limited
from numpy.testing.utils import assert_allclose


class LargeObject(object):
    """ Simulates a large object """
    def __init__(self, nbytes):
        self.nbytes = nbytes
    
    def __sizeof__(self):
        return self.nbytes
        
def class_factory(max_size, max_mem_MB, object_size):
    """ Builds example classes with different parameters """
    
    class MyClass(object):
        
        @memoize_limited(max_size=max_size, max_mem_MB=max_mem_MB)
        def f1(self, x):
            return LargeObject(object_size)
        
        @memoize_limited(max_size=max_size, max_mem_MB=max_mem_MB)
        def f2(self, x):
            return LargeObject(object_size)
        
    return MyClass
    
def expensive_test():
    MyClass = class_factory(max_size=100, max_mem_MB=100,
                            object_size=25000)
    A = MyClass()
    
    print A.f1
    
    N = 50
    for i in range(N):
        A.f1(i)
        A.f2(i)
    
    print A.memoize_cache.summary()    
    assert_allclose(A.memoize_cache.ncalls, N * 2)
    assert_allclose(A.memoize_cache.nhits, 0)
    assert_allclose(A.memoize_cache.nmiss, N * 2)
    
    for i in range(N):
        A.f1(i)
        A.f2(i)
        
    print A.memoize_cache.summary()
    assert_allclose(A.memoize_cache.ncalls, N * 4)
    assert_allclose(A.memoize_cache.nhits, N * 2)
    assert_allclose(A.memoize_cache.nmiss, N * 2)

    for i in range(N):
        A.f1(i + N)
        A.f2(i + N)
        
    print A.memoize_cache.summary()
    
    assert_allclose(A.memoize_cache.ncalls, N * 4 + N * 2)
    assert_allclose(A.memoize_cache.nhits, N * 2 + 0) # no new hits 
    assert_allclose(A.memoize_cache.nmiss, N * 2 + N * 2) # many new missed
    # we had to erase all the previous ones
    assert_allclose(A.memoize_cache.nerased, N * 2) 
    
    
if __name__ == '__main__':
    expensive_test()
