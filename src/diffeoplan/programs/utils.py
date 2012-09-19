import os
from . import logger
#from diffeoplan.utils.script_utils import UserError
#from bootstrapping_olympics.utils.wildcards import expand_string
#from contracts import contract

class Storage:
    commands = {}


def declare_command(name, short_usage=None):
    if short_usage is None:
        short_usage = name
    
    def wrap(f):
        f.short_usage = short_usage
        if name in Storage.commands:
            raise Exception('Already defined command %r.' % name)
        Storage.commands[name] = f
        return f
    return wrap


def write_report_files(report, basename):
    # TODO: hdf output
    
    filename = basename + '.html'
    dirname = os.path.dirname(filename)
    if not os.path.exists(dirname):
        os.makedirs(dirname)
        
    logger.info('Writing to %r.' % filename)
    report.to_html(filename, write_pickle=True)


