from . import UncertainImage, contract, logger
from bootstrapping_olympics.utils import safe_pickle_dump
from conf_tools.utils import friendly_path
import os
import yaml


class TestCase():
    
    @contract(id_tc='str', y0=UncertainImage, y1=UncertainImage, id_discdds='str',
              true_plan='None|list(int)')
    def __init__(self, id_tc, y0, y1, id_discdds, true_plan):
        self.id_tc = id_tc
        self.y0 = y0
        self.y1 = y1
        self.id_discdds = id_discdds
        self.true_plan = true_plan


    def save(self, outdir):
        """ Creates outdir/<id_tc>.pickle and outdir/<>.yaml """
        
        filename_pickle = self.id_tc + '.tc.pickle'
        filename_yaml = self.id_tc + '.tc.yaml' 
        description = {
                       'id': self.id_tc,
                       'desc': 'Automatically generated test case',
                       'code': [
                         'diffeoplan.library.load_pickle',
                         {'file:pickle': filename_pickle}]
                       }
        
        filename_pickle = os.path.join(outdir, filename_pickle)
        filename_yaml = os.path.join(outdir, filename_yaml)
        
        logger.info('Writing to %r ' % friendly_path(filename_pickle))
        safe_pickle_dump(self, filename_pickle)
        
        # TODO: make function in conf tools
        logger.info('Writing to %r ' % friendly_path(filename_yaml))
        with open(filename_yaml, 'w') as f:
            yaml.dump([description], f,
                      default_flow_style=False, explicit_start=True)
            
    def display(self, report):
        report.data('id_tc', self.id_tc)
        report.data('true_plan', self.true_plan)
        f = report.figure()
        f.data_rgb('y0_rgb', self.y0.get_rgb(), caption='y0 (rgb)')
        f.data_rgb('y1_rgb', self.y1.get_rgb(), caption='y1 (rgb)')
        
#    
#    
#def tc_to_yaml():
#    image1_filename = '%s-start.pickle' % tcname 
#    image2_filename = '%s-goal.pickle' % tcname
#    safe_pickle_dump(image1, os.path.join(outdir, image1_filename))
#    safe_pickle_dump(image2, os.path.join(outdir, image2_filename))
#    image1_values_filename = '%s-start.values.png' % tcname 
#    image2_values_filename = '%s-goal.values.png' % tcname
#    imwrite(image1.get_values(), os.path.join(outdir, image1_values_filename))
#    imwrite(image2.get_values(), os.path.join(outdir, image2_values_filename))
#    
#    description = {'id': tcname,
#                   'desc': 'Automatically generated test case',
#                   'code': ['diffeoplan.library.TestCase',
#                            {
#                            'file:image1_filename': image1_filename,
#                            'file:image2_filename': image2_filename,
#                            'id_discdds': id_discdds,
#                            'ground_truth': plan
#                            }
#                            ]
#                   }
#    pprint(description)
#    
 
