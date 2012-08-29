import scipy
from numpy.testing.utils import assert_allclose
from contracts import contract


@contract(x='array[HxW]', shape='tuple(A,B)', returns='array[AxB]')
def resample_signal_2d(x, shape, interp='bilinear'):
    """ Some checks around Scipy's implementation """    
    result = scipy.misc.imresize(x, shape, interp=interp, mode=None)
    assert_allclose(result.shape, shape)
    return result

@contract(x='array[HxWxM]', shape='tuple(A,B)', returns='array[AxBxM]')
def resample_signal_3d(x, shape, interp='bilinear'):
    """ Some checks around Scipy's implementation """    
    result = scipy.misc.imresize(x, shape, interp=interp, mode=None)
    assert_allclose(result.shape[:2], shape)
    return result

@contract(x='array[HxWx...]', shape='tuple(A,B)', returns='array[AxBx...]')
def resample_signal(x, shape):
    if x.ndim == 3:
        return resample_signal_3d(x, shape)
    if x.ndim == 2:
        return resample_signal_2d(x, shape)
    raise NotImplemented
