from . import (check_valid_image_config, check_valid_image,
    check_valid_dds,
    check_valid_symdiffeo_config, check_valid_symdiffeo, check_valid_symdds_config,
    check_valid_discdds_config)
from conf_tools import ConfigMaster, GenericCall, check_generic_code_desc


class DiffeoplanConfigMaster(ConfigMaster):
    def __init__(self):
        ConfigMaster.__init__(self, 'Diffeoplan')

        self.images = self.add_class('images', '*.images.yaml',
                                     check_valid_image_config,
                                     GenericCall(check_valid_image))

        self.symdiffeos = self.add_class('symdiffeos', '*.symdiffeos.yaml',
                                     check_valid_symdiffeo_config,
                                     GenericCall(check_valid_symdiffeo))

#        self.diffeos = self.add_class('diffeos', '*.diffeos.yaml',
#                                     check_valid_diffeo_config,
#                                     GenericCall(check_valid_diffeo))
# 
        self.symdds = self.add_class('symdds', '*.symdds.yaml',
                                     check_valid_symdds_config,
                                     GenericCall(check_valid_dds))

        self.discdds = self.add_class('discdds', '*.discdds.yaml',
                                     check_valid_discdds_config,
                                     GenericCall(check_valid_dds))
  
        self.algos = self.add_class('algos', '*.algos.yaml',
                                     check_generic_code_desc,
                                     GenericCall())
 
        self.testcases = self.add_class('testcases', '*.tc.yaml',
                                     check_generic_code_desc,
                                     GenericCall())
 
 
    def get_default_dir(self):
        from pkg_resources import resource_filename #@UnresolvedImport
        return resource_filename("diffeoplan", "configs")



#DiffeoplanConfig = DiffeoplanConfigMaster()
#DiffeoplanConfig.load()


