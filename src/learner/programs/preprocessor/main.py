from . import PreProcessor, logger
from optparse import OptionParser
from diffeoplan.utils.script_utils import UserError, wrap_script_entry_point
import os
from conf_tools.exceptions import ConfToolsException #@UnresolvedImport


def preprocessor(args):
    usage = "usage: %prog -i inputpath [-o outputdir] -s [W,H] -p string"
    parser = OptionParser(usage=usage, version="%prog 1.0")
    parser.add_option("-i", "--input",
                      help="Path to input file (must be *.raw.bag)")
    parser.add_option("-o", "--output",
                      help="Path to output directory")
    parser.add_option("-p", "--namefix", default='',
                      help='Additional string for output filename')
    parser.add_option("-s", "--size", default='[160,120]', help="Image size WxH")
    parser.add_option("-z", "--zoom", default=True, help="Use zoom")
#    parser.add_option("-zc", "--zoom_center", default=[0,0], help="Not Impl: Zoom center offset from mid image")


    options, other = parser.parse_args(args)
    if other:
        raise UserError('Spurious arguments %r.' % other)

    bag = options.input 
    
    if bag is None:
        msg = 'Must specify input file with -i.'
        raise UserError(msg)

    if not os.path.exists(bag):
        msg = 'Filename %r does not exist.' % bag
        raise UserError(msg)
    
    input_suffix = '.raw.bag'
    
    if not input_suffix in bag:
        msg = 'I expect a %s as input, got %r.' % (input_suffix, bag)
        raise UserError(msg)
    
    if options.output is None:
        outdir = os.path.dirname(bag)
        msg = 'No output dir specified; I will use %s' % outdir
        logger.info(msg)
    else:
        outdir = options.output
        
    basename = os.path.basename(bag)    
    basename = basename[:-len(input_suffix)]

    output = os.path.join(outdir, basename + '.processed.bag')

    logger.info('Reading from: %s' % bag)
    logger.info('Writing to:   %s' % output)    
    output_size = eval(options.size)
    
    zoom = eval(options.zoom)
    
    pproc = PreProcessor(bag, output, output_size, use_zoom=zoom)
    pproc.process_bag()
    pproc.finalize()


def preprocessor_main():
    exceptions_no_traceback = (UserError, ConfToolsException)
    return wrap_script_entry_point(preprocessor, logger,
                        exceptions_no_traceback=exceptions_no_traceback)

if __name__ == '__main__':
    preprocessor_main()
