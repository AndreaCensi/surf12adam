from . import logger
from .. import declare_command

@declare_command('show-stream',
                 'show-stream  [<stream1> ...]')
def show_stream(config, parser): #@UnusedVariable
    """ Displays the learned DDS """
    parser.add_option("-o", "--output", help="Output directory",
                      default='out/dp-show-stream/')
    options, which = parser.parse()
  
    # TODO: compute and display statistics  
    outdir = options.output 
  
    if not which:
        todo = config.streams.keys()  
    else:
        todo = config.streams.expand_names(which)


    for id_stream in todo:
        stream = config.streams.instance(id_stream)
        read_stream(id_stream, stream)
        

def read_stream(id_stream, stream):
    logger.info('Reading all data in stream %r.' % id_stream)
    
    nrecords = 0
    for log_item in stream.read_all(): #@UnusedVariable
        nrecords += 1
    logger.info('Total of %d records' % nrecords)

