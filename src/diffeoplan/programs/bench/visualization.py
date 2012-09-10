from . import np
from reprep import Report, scale, rgb_zoom
from diffeoplan.configuration import set_current_config


def visualize_result(config, id_tc, id_algo, stats):
    """ Returns a report """
    set_current_config(config)
    
    result = stats['result']
    r = Report('%s-%s' % (id_tc, id_algo))
#    tc = config.testcases.instance(id_tc)
#    discdds = config.discdds.instance(tc.id_discdds)
    algo = stats['algo']
    tc = stats['tc']
    discdds = algo.get_dds()
    
    tc.display(r.section('testcase'), discdds=discdds)
    
    if not result.success:
        r.text('warning', 'Plannning unsuccesful')
    else:
        rsol = r.section('solution')
        rsol.text('plan', 'Plan: %s' % str(result.plan))
    
        y0 = tc.y0
        y1 = tc.y1
        y1plan = discdds.predict(y0, result.plan)
        mismatch = np.abs(y1.get_values() - y1plan.get_values()).sum(axis=2)
        
        f = rsol.figure(cols=4)
        zoom = lambda x: rgb_zoom(x, 8)
        
        f.data_rgb('y1plan', zoom(y1plan.get_rgb()),
                   caption='plan prediction (certain)')
        f.data_rgb('y1plan_certain', zoom(y1plan.get_rgb_uncertain()),
                   caption='certainty of prediction')
        
        f.data_rgb('mismatch', zoom(scale(mismatch)),
                   caption='Mismatch value pixel by pixel '
                            '(zero for synthetic testcases...)')
    
    
    algo.plan_report(r.section('planning'), result, tc)
    
    extra = result.extra
    
    write_log_lines(r, extra)
    
    return r


def write_log_lines(r, extra):    
    if 'log_lines' in extra:
        log_lines = extra['log_lines']
        lines = log_lines
        r.text('execution_log', '\n'.join(lines)) # XXX
    else:
        r.text('execution_log', '(no log recorded)')
    
    
