from . import logger
from .. import declare_command
from bootstrapping_olympics.utils import raise_x_not_found, safe_pickle_dump
from conf_tools.utils import friendly_path
from contracts import describe_type
from diffeoplan.library import DiffeoSystem
import os
import yaml

@declare_command('makedds',
                 'makedds [<dds1> <dds2> ...]')
def symdds_make_main(config, parser): #@UnusedVariable
    parser.add_option("-o", "--output", default='out/dds-generated',
                      help="Output directory")
    options, which = parser.parse()
    
    outdir = options.output

    if not which:
        which = sorted(config.symdds.keys())
        
    make_all_dds(config, which, outdir)
    
def make_all_dds(config, which, outdir):
    if not os.path.exists(outdir):
        os.makedirs(outdir) 
        
    for id_symdds in which:
        if not id_symdds in config.symdds:
            raise_x_not_found('symdds', id_symdds, config.symdds)
                
    for id_symdds in which:
        make_dds(config, id_symdds, outdir)
        
def make_dds(config, id_symdds, outdir):
    dds = instance_dds(config, id_symdds)
    basename = os.path.join(outdir, id_symdds)
    filename_pickle = basename + '.discdds.pickle'
    logger.info('Writing to %r ' % friendly_path(filename_pickle))
    safe_pickle_dump(dds, filename_pickle)
            
    filename_yaml = basename + '.discdds.yaml'
    description = {
        'id': id_symdds,
        'desc': 'Synthetically generated from symbolic DDS %r.' % id_symdds,
        'code': ['diffeoplan.library.load_pickle',
                 {'file:pickle': id_symdds + '.discdds.pickle'}]
    }
    logger.info('Writing to %r ' % friendly_path(filename_yaml))
    with open(filename_yaml, 'w') as f:
        yaml.dump([description], f, default_flow_style=False, explicit_start=True)
        
        
def instance_dds(config, id_symdds): 
    dds = config.symdds.instance(id_symdds) #@UndefinedVariable
    if not isinstance(dds, DiffeoSystem):
        msg = 'I expect to find a DDSFromSymbolic, not %r' % describe_type(dds)
        raise ValueError(msg)
    dds.label = id_symdds
    return dds
    
