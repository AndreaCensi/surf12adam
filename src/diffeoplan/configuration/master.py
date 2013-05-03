from . import (check_valid_image_config, check_valid_image, check_valid_dds,
    check_valid_symdiffeo_config, check_valid_symdiffeo, check_valid_symdds_config,
    check_valid_discdds_config)
from conf_tools import ConfigMaster, GenericCall, check_generic_code_desc
from contracts import contract
from diffeoplan.configuration.checks import check_valid_set


class DiffeoplanConfigMaster(ConfigMaster):
    def __init__(self):
        ConfigMaster.__init__(self, 'dp')

        self.images = self.add_class('images', '*.images.yaml',
                                     check_valid_image_config,
                                     GenericCall(check_valid_image))

        self.symdiffeos = self.add_class('symdiffeos', '*.symdiffeos.yaml',
                                     check_valid_symdiffeo_config,
                                     GenericCall(check_valid_symdiffeo))

        self.symdds = self.add_class('symdds', '*.symdds.yaml',
                                     check_valid_symdds_config,
                                     GenericCall(check_valid_dds))

        self.discdds = self.add_class('discdds', '*.discdds.yaml',
                                     check_valid_discdds_config,
                                     GenericCall(check_valid_dds))
  
        self.algos = self.add_class('algos', '*.algos.yaml',
                                     check_generic_code_desc,
                                     GenericCall())
        
        self.distances = self.add_class('distances', '*.distances.yaml',
                                     check_generic_code_desc,
                                     GenericCall())
 
        self.testcases = self.add_class('testcases', '*.tc.yaml',
                                     check_generic_code_desc,
                                     GenericCall())

        self.streams = self.add_class('streams', '*.streams.yaml',
                                     check_generic_code_desc,
                                     GenericCall())
        
        self.learners = self.add_class('learners', '*.learners.yaml',
                                     check_generic_code_desc,
                                     GenericCall())
        
        self.online_testcases = self.add_class('online_testcases', '*.online.yaml',
                                     check_generic_code_desc,
                                     GenericCall())
 
        self.sets = self.add_class('sets', '*.batch.yaml', check_valid_set)
  
  
    def get_default_dir(self):
        from pkg_resources import resource_filename  # @UnresolvedImport
        return resource_filename("diffeoplan", "configs")

    singleton = None



@contract(returns=DiffeoplanConfigMaster)
def get_dp_config():
    if DiffeoplanConfigMaster.singleton is None:
        DiffeoplanConfigMaster.singleton = DiffeoplanConfigMaster()
    return DiffeoplanConfigMaster.singleton 


@contract(c=DiffeoplanConfigMaster)
def set_dp_config(c):
    DiffeoplanConfigMaster.singleton = c  






