from . import logger
from .. import declare_command
from bootstrapping_olympics.utils import raise_x_not_found, safe_pickle_dump
from conf_tools.utils import friendly_path
from contracts import describe_type
from diffeoplan.library import DiffeoSystem
import os
import yaml
from diffeoplan.configuration.master import DiffeoplanConfig

@declare_command('symdds-make',
                 'symdds-make [<dds1> <dds2> ...]')
def symdiffeo_display_main(global_config, parser): #@UnusedVariable
    parser.add_option("-o", "--output", default='out/dds-generated',
                      help="Output directory")
    options, which = parser.parse()
    
    outdir = options.output
    config = DiffeoplanConfig

    if not which:
        which = sorted(config.symdds.keys())
        
    make_all_dds(which, outdir)
    
def make_all_dds(which, outdir):
    if not os.path.exists(outdir):
        os.makedirs(outdir)

    config = DiffeoplanConfig

    for id_dds in which:
        if not id_dds in config.symdds:
            raise_x_not_found('symdds', id_dds, config.dds)
                
    for id_dds in which:
        make_dds(id_dds, outdir)
        
def make_dds(id_dds, outdir):
    dds = instance_dds(id_dds)
    
    basename = os.path.join(outdir, id_dds)
    filename_pickle = basename + '.pickle'
    logger.info('Writing to %r ' % friendly_path(filename_pickle))
    safe_pickle_dump(dds, filename_pickle)
            
    filename_yaml = basename + '.discdds.yaml'
    description = {
        'id': id_dds,
        'desc': 'Synthetic dataset',
        'code': ['diffeoplan.library.load_pickle',
                 {'file:pickle': id_dds + '.pickle'}]
    }
    logger.info('Writing to %r ' % friendly_path(filename_yaml))
    with open(filename_yaml, 'w') as f:
        yaml.dump([description], f, default_flow_style=False, explicit_start=True)
        
        
def instance_dds(id_dds):
    dds = DiffeoplanConfig.symdds.instance(id_dds) #@UndefinedVariable
    if not isinstance(dds, DiffeoSystem):
        msg = 'I expect to find a DDSFromSymbolic, not %r' % describe_type(dds)
        raise ValueError(msg)
    dds.label = id_dds
    return dds
    
