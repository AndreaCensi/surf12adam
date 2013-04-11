from . import  logger
from optparse import OptionParser
from diffeoplan.utils.script_utils import UserError, wrap_script_entry_point
import os
from conf_tools.exceptions import ConfToolsException #@UnresolvedImport
from learner.programs.preprocessor.pre_processor import preprocess, \
    preprocess_general


def preprocessor(args):
    usage = "usage: %prog -i inputname [-o outputdir] -s [W,H] -z bool"
    parser = OptionParser(usage=usage, version="%prog 1.0")
    parser.add_option("-d", "--device", default="orbit",
                      help="Type of the device the raw data come from.")
    
    parser.add_option("-i", "--input",
                      help="Path and name to input file (must be *.raw.bag)")
    parser.add_option("-o", "--output",
                      help="Path to output directory")
    parser.add_option("-s", "--size", default='[160,120]', help="Image size WxH")
    parser.add_option("-z", "--zoom", default=None, help="Limits for zoom values and scaling factor for zoomer")


    options, other = parser.parse_args(args)
    if other:
        raise UserError('Spurious arguments %r.' % other)

    if options.device == 'orbit':
        preprocessor_orbit(options)
    elif options.device == 'general':
        preprocessor_general(options)

def preprocessor_orbit(options):
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
    
    info_list = basename.split('-')
    
    # If zoom property is given, then use zoom. Evaluate zoomer parameters
    if hasattr(options, 'zoom'):
        try:
            vals = eval(options.zoom)
            min_zoom = vals[0]
            max_zoom = vals[1]
            zoom_factor = vals[2]
            zoom = True
        except:
            # 
            zoom = False
    else:
        zoom = False
        # if zoom is not used, then remove z* from command name
        info_list[1] = info_list[1][:info_list[1].index('z')] 
#        out_info = '_' + str(output_size).replace(' ', '').replace(',', 'x').replace('(', '').replace(')', '') + '_nozoom'

    output_size = tuple(eval(options.size))
    # prepare the size for the output name
    info_list.append(str(output_size).strip('()').replace(', ', 'x'))
    
    # start build the output name
    out_name = info_list[0]
    # remove the robot name from list
    info_list.remove(info_list[0])
    
    for part in info_list:
        out_name += '-' + part
    
    output = os.path.join(outdir, out_name + '.processed.bag')

    logger.info('Reading from: %s' % bag)
    logger.info('Writing to:   %s' % output)
    if zoom:
        preprocess(bag, output, output_size,
                   use_zoom=zoom,
                   min_zoom=min_zoom, max_zoom=max_zoom, zoom_factor=zoom_factor)
    else:
        preprocess(bag, output, output_size,
                   use_zoom=False)
    # pproc.validate_bag(output)

def preprocessor_general(options):
    '''
    Analysis the data types in the raw bag to determine which are the commands
    :param options:
    '''
    
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
    
    output_size = tuple(eval(options.size))
    
    
    out_name = basename
    
    output = os.path.join(outdir, out_name + '.processed.bag')

    logger.info('Reading from: %s' % bag)
    logger.info('Writing to:   %s' % output)
    
    preprocess_general(bag, output, output_size)

def preprocessor_main():
    exceptions_no_traceback = (UserError, ConfToolsException)
    return wrap_script_entry_point(preprocessor, logger,
                        exceptions_no_traceback=exceptions_no_traceback)

if __name__ == '__main__':
    preprocessor_main()
