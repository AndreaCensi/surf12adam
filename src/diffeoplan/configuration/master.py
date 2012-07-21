from . import (check_valid_image_config, check_valid_image,
    check_valid_diffeo_config, check_valid_diffeo, check_valid_dds_config,
    check_valid_dds)
from conf_tools import ConfigMaster, GenericCall


class DiffeoplanConfigMaster(ConfigMaster):
    def __init__(self):
        ConfigMaster.__init__(self, 'BootOlympics')

        self.images = self.add_class('images', '*.images.yaml',
                                     check_valid_image_config,
                                     GenericCall(check_valid_image))

        self.diffeos = self.add_class('diffeos', '*.diffeos.yaml',
                                     check_valid_diffeo_config,
                                     GenericCall(check_valid_diffeo))
 
        self.dds = self.add_class('dds', '*.dds.yaml',
                                     check_valid_dds_config,
                                     GenericCall(check_valid_dds))
 
    def get_default_dir(self):
        from pkg_resources import resource_filename #@UnresolvedImport
        return resource_filename("diffeoplan", "configs")


DiffeoplanConfig = DiffeoplanConfigMaster()
DiffeoplanConfig.load()
print 'here'

