import copy
import numpy as np
from boot_agents.diffeo.diffeomorphism2d import Diffeomorphism2D
from boot_agents.diffeo.diffeo_display import diffeo_stats


def make_hard_choices(dds, info_threshold=0.5, use_isomorphism_heuristics=True,
                      info_percentile=50):
    """ Applies a threshold to the uncertainty of the diffeomorphisms """ 
    dds = copy.deepcopy(dds)
    
    def make_hard(dd):
        assert isinstance(dd, Diffeomorphism2D)
        if use_isomorphism_heuristics:
            stats = diffeo_stats(dd.d)
            limit = np.percentile(stats.norm, info_percentile) / 3.0
            #print('norm mean/mean: %g %g' % (np.mean(stats.norm), np.median(stats.norm)))            
            #for i in range(0, 100, 5):
            #    print(' %3d%% = %g' % (i, np.percentile(stats.norm, i)))
            #limit = np.percentile(stats.norm, info_percentile)
            #if limit <= 1:
            #    print('limit was %g' % limit)
            #    limit = 4
            dd.variance = (stats.norm > limit).astype('float')
            print('limit: %g pixels' % limit)
            print('visibility: %g' % np.mean(dd.variance))
        else:
            dd.variance = (dd.variance > info_threshold).astype('float')
    for a in dds.actions:
        df = a.get_diffeo2d_forward()
        make_hard(df)
        db = a.get_diffeo2d_backward()
        make_hard(db)
    return dds
