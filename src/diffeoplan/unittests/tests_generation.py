 

from bootstrapping_olympics.unittests.utils import fancy_test_decorator
from diffeoplan.configuration import get_current_config, Storage, \
    set_current_config
from diffeoplan.configuration.master import DiffeoplanConfigMaster

def testconfig():
    if Storage.config is None:
        config = DiffeoplanConfigMaster()
        config.load()
        set_current_config(config)
        
    return get_current_config()

#
#for_all_diffeos = fancy_test_decorator(lister=lambda: testconfig().diffeos.keys(),
#            arguments=lambda id_diffeo: (id_diffeo, testconfig().diffeos.instance(id_diffeo)),
#            attributes=lambda id_diffeo: dict(diffeo=id_diffeo),
#            debug=True)

for_all_symdiffeos = fancy_test_decorator(lister=lambda: testconfig().symdiffeos.keys(),
            arguments=lambda id_symdiffeos: 
                (id_symdiffeos, testconfig().symdiffeos.instance(id_symdiffeos)),
            attributes=lambda id_symdiffeos: dict(symdiffeos=id_symdiffeos),
            debug=True)

                                                 
for_all_images = fancy_test_decorator(lister=lambda: testconfig().images.keys(),
            arguments=lambda id_image: (id_image, testconfig().images.instance(id_image)),
            attributes=lambda id_image: dict(image=id_image),
            debug=True)

for_all_dds = fancy_test_decorator(lister=lambda: testconfig().discdds.keys(),
            arguments=lambda id_dds: (id_dds, testconfig().discdds.instance(id_dds)),
            attributes=lambda id_dds: dict(dds=id_dds),
            debug=True)

#
#@for_all_diffeos
#def check_diffeo_dummy(id_diffeo, diffeo):
#    pass


@for_all_symdiffeos
def check_symdiffeo_dummy(id_symdiffeo, symdiffeo):
    pass


@for_all_dds
def check_dds_dummy(id_dds, dds):
    pass


@for_all_images
def check_image_dummy(id_image, image):
    pass
