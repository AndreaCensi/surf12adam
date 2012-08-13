from . import  contract
from diffeoplan.library import TestCase, UncertainImage
from procgraph_pil import resize


def discdds_make_test_cases(config, outdir, id_discdds,
                            id_image, num_cases, plan_length):
    # Get the DiffeoSystem
    discdds = config.discdds.instance(id_discdds)
    # Get the RGB image
    rgb = config.images.instance(id_image)
    
    shape = discdds.get_shape()
    rgb = resize(rgb, shape[1], shape[0])
        
    assert rgb.shape[0] == shape[0]
    assert rgb.shape[1] == shape[1]
    
    image = UncertainImage(rgb)
    
    for i in range(num_cases):
        tcname = 'tc_%s_%s_%02d_%03d' % (id_discdds, id_image, plan_length, i)
        discdds_make_test_case(outdir, tcname, id_discdds,
                               discdds, image, plan_length)


@contract(image1=UncertainImage)
def discdds_make_test_case(outdir, tcname, id_discdds,
                           discdds, image1, plan_length):
    # Get a random plan
    plan = discdds.get_random_plan(plan_length)
    # predict the result
    image2 = discdds.predict(image1, plan)
    
    tc = TestCase(id_tc=tcname, id_discdds=id_discdds,
                  y0=image1, y1=image2, true_plan=plan)
    tc.save(outdir)
    
    
    
