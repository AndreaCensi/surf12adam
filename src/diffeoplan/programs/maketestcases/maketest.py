from . import logger
from conf_tools.utils import friendly_path
from diffeoplan.configuration.master import DiffeoplanConfig
from procgraph_pil.imwrite import imwrite
import os
import yaml
from diffeoplan.library.uncertain_image import UncertainImage
from contracts import contract
from bootstrapping_olympics.utils.safe_pickle import safe_pickle_dump
from procgraph_pil.pil_operations import resize
from pprint import pprint

def discdds_make_test_cases(outdir, id_discdds, id_image, num_cases, plan_length):
    config = DiffeoplanConfig
    # Get the DiffeoSystem
    discdds = config.discdds.instance(id_discdds)
    # Get the RGB image
    rgb = config.images.instance(id_image)
    # resize if necessary
#    h, w = rgb.shape[0], rgb.shape[1]
    shape = discdds.get_shape()
    rgb = resize(rgb, shape[0], shape[1])
    image = UncertainImage(rgb)
    
    for i in range(num_cases):
        tcname = 'tc_%s_%s_%02d_%03d' % (id_discdds, id_image, plan_length, i)
        discdds_make_test_case(outdir, tcname, id_discdds, discdds, image, plan_length)

@contract(image1=UncertainImage)
def discdds_make_test_case(outdir, tcname, id_discdds, discdds, image1, plan_length):
    # Get a random plan
    plan = discdds.get_random_plan(plan_length)
    # predict the result
    image2 = discdds.predict(image1, plan)
    # write everything
    image1_filename = '%s-start.pickle' % tcname 
    image2_filename = '%s-goal.pickle' % tcname
    safe_pickle_dump(image1, os.path.join(outdir, image1_filename))
    safe_pickle_dump(image2, os.path.join(outdir, image2_filename))
    image1_values_filename = '%s-start.values.png' % tcname 
    image2_values_filename = '%s-goal.values.png' % tcname
    imwrite(image1.get_values(), os.path.join(outdir, image1_values_filename))
    imwrite(image2.get_values(), os.path.join(outdir, image2_values_filename))
    
    description = {'id': tcname,
                   'desc': 'Automatically generated test case',
                   'code': ['diffeoplan.library.TestCase',
                            {
                            'file:image1_filename': image1_filename,
                            'file:image2_filename': image2_filename,
                            'id_discdds': id_discdds,
                            'ground_truth': plan
                            }
                            ]
                   }
    pprint(description)
    filename_yaml = os.path.join(outdir, '%s.tc.yaml' % tcname)
    logger.info('Writing to %r ' % friendly_path(filename_yaml))
    with open(filename_yaml, 'w') as f:
        yaml.dump([description], f,
                  default_flow_style=False, explicit_start=True)
    
    
