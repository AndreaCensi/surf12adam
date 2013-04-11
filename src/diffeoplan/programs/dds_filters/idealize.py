'''
Created on Feb 28, 2013

@author: adam
'''
from diffeoplan.library.discdds.writing import ds_dump
from diffeoplan.programs.utils import declare_command
from reprep import Report
from reprep.report_utils import ReportManager
import itertools
import numpy as np
import numpy.linalg as la
import os
from compmake import (batch_command, compmake_console, comp, use_filesystem,
                      read_rc_files)
import copy

@declare_command('idealize-uncert',
                 'idealize-uncert -S discdds -o [output and input folder]')
def idealize_uncert(config, parser):
    '''
    Assumes a constant displacement over the whole sensor domain
    '''
    parser.add_option("-S", "--dds", help="DDS sytem .")
    parser.add_option("-c", "--command", \
                      help="Command to pass to compmake for batch mode")
    parser.add_option("-o", "--output", help="Output directory", \
                      default='out/idealized-dds/')
    options = parser.parse_options()



    id_discdds = options.dds
    dds = config.discdds.instance(id_discdds)

    outdir = os.path.join(options.output, id_discdds)
    
    storage = os.path.join(outdir, 'compmake')
    use_filesystem(storage)
    read_rc_files()
    
#   
    rm = ReportManager(os.path.join(outdir, 'reports')) 
    
    # Ideal
    id_iu_discdds = 'iu-' + id_discdds
    iu_dds = comp(_idealize_uncert, dds, job_id='idealize_uncert')
    comp(save_results, id_iu_discdds, outdir, iu_dds)
    diffeo_report = comp(report_dds, 'iu_dds-%s' % id_discdds, iu_dds)
    rm.add(diffeo_report, 'iu-dds', id_learner='idealized-uncertainty')
              
    # Relative  
    id_uur_discdds = 'uur-' + id_discdds
    dds_copyr = copy.copy(dds)
    uur_dds = comp(_update_uncert, dds_copyr, length_score_norm_relative,
                   job_id='update_uncert_relative')
    comp(save_results, id_uur_discdds, outdir, uur_dds,
                         job_id='update_uncert_relative_save')
    diffeo_report = comp(report_dds, 'uur-dds-%s' % id_discdds, uur_dds,
                         job_id='update_uncert_relative_report')
    rm.add(diffeo_report, 'uur-dds', id_learner='updated-uncertainty-uur')
    
    # Absolute
    dds_copya = copy.copy(dds)
    id_uua_discdds = 'uua-' + id_discdds
    uua_dds = comp(_update_uncert, dds_copya, length_score_norm,
                   job_id='update_uncert_absolute')
    comp(save_results, id_uua_discdds, outdir, uua_dds,
                         job_id='update_uncert_absolute_save')
    diffeo_report = comp(report_dds, 'uua-dds-%s' % id_discdds, uua_dds,
                         job_id='update_uncert_absolute_report')
    rm.add(diffeo_report, 'uua-dds', id_learner='updated-uncertainty-uua')
    
    
    rm.create_index_job()
    
    if options.command:
        return batch_command(options.command)
    else:
#        pdb.set_trace()
        compmake_console()
        return 0
    
    
def _idealize_uncert(dds):
    for action in dds.actions:
        field = action.diffeo.d
        field_inv = action.diffeo_inv.d
    
        I = np.zeros(field.shape)
        Y, X = np.meshgrid(range(field.shape[1]), range(field.shape[0]))
        I[:, :, 0] = X
        I[:, :, 1] = Y
        
        D = field - I
        v = (np.median(D[:, :, 0]), np.median(D[:, :, 1]))
        
        D_inv = field_inv - I
        v_inv = (np.median(D_inv[:, :, 0]), np.median(D_inv[:, :, 1]))
        
        print('v     = ' + str(v))
        print('v_inv = ' + str(v_inv))
        
        for c in itertools.product(range(X.shape[0]), range(X.shape[1])):
            
            if defined_cell(c, X.shape, v):
                action.diffeo.variance[c] = 1.0
            else:
                action.diffeo.variance[c] = 0.0
                
            if defined_cell(c, X.shape, v_inv):
                action.diffeo_inv.variance[c] = 1.0
            else:
                action.diffeo_inv.variance[c] = 0.0
    return dds
    
#@declare_command('update-uncert',
#                 'update-uncert -S discdds -o [output and input folder]')  
#def update_uncert(config, parser):
#    parser.add_option("-S", "--dds", help="DDS sytem .")
#    parser.add_option("-c", "--command", \
#                      help="Command to pass to compmake for batch mode")
#    parser.add_option("-o", "--output", help="Output directory", \
#                      default='out/idealized-dds/')
#    options = parser.parse_options()
#
#    id_discdds = options.dds
#    dds = config.discdds.instance(id_discdds)
#    
def _update_uncert(dds, length_score):
    for action in dds.actions:
        action.update_uncertainty(length_score)
    return dds

    
#@contract(returns='>=0')
def length_score_norm(v, v_inv):
    '''
        
    '''
#    l = la.norm(v)
#    l_inv = la.norm(v_inv)

    return la.norm(np.array(v) + v_inv) 
    
#@contract(returns='>=0')
def length_score_norm_relative(v, v_inv):
    '''
        
    '''
    l = la.norm(v)
    l_inv = la.norm(v_inv)
    l_mean = 0.5 * (l + l_inv)
    if l_mean == 0:
        return 0
    else:
        return la.norm(np.array(v) + v_inv) / l_mean

def empty_report():
    return Report('empty')

def defined_cell(c, shape, v):
    # Check conditions which each one are necessary to be defined
    if v[0] <= 0 and c[0] < -v[0]:
        return False
    if v[1] <= 0 and c[1] < -v[1]:
        return False
    
    if v[0] > 0 and c[0] > shape[0] - v[0]:
        return False
    if v[1] > 0 and c[1] > shape[1] - v[1]:
        return False
    
    # if we get here, we return true
    return True
    
def report_dds(id_report, dds):
    r = Report(id_report)
    dds.display(r)
    return r
        

def save_results(name, outdir, dds):
    id_dds = name
    resdir = os.path.join(outdir, 'results')
    desc = "Idealized from %s" % name
    ds_dump(dds, resdir, id_dds, desc)
