from . import  logger
from optparse import OptionParser
from diffeoplan.utils.script_utils import UserError, wrap_script_entry_point
import os
from conf_tools.exceptions import ConfToolsException #@UnresolvedImport
from learner.programs.preprocessor.pre_processor import preprocess


def preprocessor(args):
    usage = "usage: %prog -i inputname [-o outputdir] -s [W,H] -z bool"
    parser = OptionParser(usage=usage, version="%prog 1.0")
    parser.add_option("-i", "--input",
                      help="Path and name to input file (must be *.raw.bag)")
    parser.add_option("-o", "--output",
                      help="Path to output directory")
    parser.add_option("-s", "--size", default='[160,120]', help="Image size WxH")
    parser.add_option("-z", "--zoom", default='True', help="Use zoom (True, False)")


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
    
    info_list = basename.split('-')
    
    zoom = eval(options.zoom)

    # if zoom is not used, then remove z* from command name
    if zoom:
        pass
        #out_info = '_' + str(output_size).replace(' ', '').replace(',', 'x').replace('(', '').replace(')', '') + '_zoom'
    else:
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
    
    preprocess(bag, output, output_size,
                use_zoom=zoom,
                min_zoom=100, max_zoom=200)
    
    # pproc.validate_bag(output)


def preprocessor_main():
    exceptions_no_traceback = (UserError, ConfToolsException)
    return wrap_script_entry_point(preprocessor, logger,
                        exceptions_no_traceback=exceptions_no_traceback)

if __name__ == '__main__':
    preprocessor_main()
