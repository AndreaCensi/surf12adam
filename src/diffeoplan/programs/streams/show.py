from . import np, contract, logger
from .. import declare_command, write_report_files
from diffeoplan.library.images.uncertain_image import UncertainImage
from reprep import Report
import os


@declare_command('show-stream',
                 'show-stream  [<stream1> ...]')
def show_stream(config, parser): #@UnusedVariable
    """ Displays the learned DDS """
    #parser.add_option("-i", "--id_image", help="ID image.", default='lena')
    parser.add_option("-o", "--output", help="Output directory",
                      default='out/show_stream/')
    options, which = parser.parse()
    
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
    for y0, u, y1 in stream.read_all():
#            u = tuple(u.aslist())
#        print('%s' % str(u))
        nrecords += 1
    logger.info('Total of %d records' % nrecords)

#        
#    id_image = options.id_image
#    image = UncertainImage(config.images.instance(id_image))
#    
#    for id_dds in todo:
#        dds = config.discdds.instance(id_dds) 
#        report = Report(id_dds)
#        
#        dds.display(report, image=image)
#    
#        write_report_files(report, basename=os.path.join(outdir, id_dds))    
#
