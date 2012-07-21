from .. import DiffeoplanConfig
from bootstrapping_olympics.unittests.utils.generation import fancy_test_decorator

config = DiffeoplanConfig

for_all_diffeos = fancy_test_decorator(lister=config.diffeos.keys,
            arguments=lambda id_diffeo: (id_diffeo, config.diffeos.instance(id_diffeo)),
            attributes=lambda id_diffeo: dict(diffeo=id_diffeo),
            debug=True)

                                                 
for_all_images = fancy_test_decorator(lister=config.images.keys,
            arguments=lambda id_image: (id_image, config.images.instance(id_image)),
            attributes=lambda id_image: dict(image=id_image),
            debug=True)

for_all_dds = fancy_test_decorator(lister=config.dds.keys,
            arguments=lambda id_dds: (id_dds, config.dds.instance(id_dds)),
            attributes=lambda id_dds: dict(dds=id_dds),
            debug=True)


@for_all_diffeos
def check_diffeo_dummy(id_diffeo, diffeo):
    pass


@for_all_dds
def check_dds_dummy(id_dds, dds):
    pass


@for_all_images
def check_image_dummy(id_image, image):
    pass
