'''
Created on Feb 14, 2013

@author: adam
'''
import os
from . import logger
from compmake import (batch_command, compmake_console, comp, read_rc_files,
    use_filesystem)
from diffeoplan.programs.utils import declare_command
from reprep.report_utils import ReportManager
import pdb
from diffeoplan.library.images import UncertainImage
import numpy as np
import pylab

@declare_command('uncert', 'uncert  dds <stream1> [...]')
def uncert(config, parser): #@UnusedVariable
    parser.add_option("-s", "--streams", help="Which streams to use.",
                      default="*")
    parser.add_option("-S", "--dds", help="DDS sytem .")
    parser.add_option("-o", "--output", help="Output directory",
                      default='out/dp-uncert/')
    parser.add_option("-c", "--command",
                      help="Command to pass to compmake for batch mode")
    options = parser.parse_options()
     
    dds = config.discdds.expand_names(options.dds) 
    streams = config.streams.expand_names(options.streams)
    
    id_comb = ",".join(dds) + "-" + ",".join(streams)
    
    outdir = os.path.join(options.output, id_comb) 
    storage = os.path.join(outdir, 'compmake')
    use_filesystem(storage)
    read_rc_files()
    
    
    rm = ReportManager(os.path.join(outdir, 'reports'))
    
    for dds_id in dds:
        uncert_stats(config, dds_id, streams, outdir)
        
    
    rm.create_index_job()
    
    if options.command:
        return batch_command(options.command)
    else:
        compmake_console()
        return 0

def uncert_stats(config, dds_id, streams, outdir):
    
    for stream_id in streams:
        comp(calculate_uncert_stats, config, dds_id, stream_id, outdir)

def calculate_uncert_stats(config, dds_id, stream_id, outdir):
    dds = config.discdds.instance(dds_id)
    stream = config.streams.instance(stream_id)
    metric = config.distances.instance('L2')
    
    logitems = stream.read_all_state()
    
    for i, (y0, u, y1, x0) in enumerate(logitems):
#        pdb.set_trace()
        Y0 = UncertainImage(y0)
        Y1 = UncertainImage(y1)
        U0 = dds.command_to_index(u)
        uimg = dds.predict(Y0, [U0])
        
        for j in [0, 1, 2, 3]:
            pylab.figure()
            pylab.imshow(np.sum(dds.predict(Y0, [j]).get_values() - Y0.get_values(), axis=2))
            pylab.colorbar()
            pylab.savefig(outdir + '/diff_' + str(j) + '.png')
            pylab.clf()
        
        
        
#        diff0 = metric.distance(Y0, uimg)
#        diff1 = metric.distance(Y1, uimg)
#        
#        logger.info(diff0, diff1)
        
        diff = uimg.get_values() - UncertainImage(y1).get_values()
        diff_0 = uimg.get_values() - UncertainImage(y0).get_values()
        variance = dds.actions[U0].diffeo.variance
        diff = np.sum(diff, axis=2)
        variance = variance / np.mean(variance)
        diff = diff / np.mean(diff)
        
        res = variance - diff
        logger.info('  mean(res) : ' + str(np.mean(res)))
        logger.info('  std(res) : ' + str(np.std(res)))
        
        pylab.figure()
        pylab.imshow(diff, vmin= -15, vmax=15)
        pylab.savefig(outdir + 'diff' + str(i) + '.png')
        pylab.clf()
        pylab.imshow(diff, vmin= -15, vmax=15)
        pylab.savefig(outdir + 'diff' + str(i) + '.png')
        pylab.clf()
        pylab.close()
        pdb.set_trace()
