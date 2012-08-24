from diffeoplan.configuration import get_current_config
from diffeoplan.library.images.test_case import TestCase
from diffeoplan.library.images.uncertain_image import UncertainImage
from boot_agents.diffeo.analysis.pil_utils import resize
from geometry.utils.numpy_backport import assert_allclose



def ManualMotion(tcname, id_discdds, id_image, planstring):
    # Get a random plan
    config = get_current_config()
    discdds = config.discdds.instance(id_discdds)
    rgb = config.images.instance(id_image)
    shape = discdds.get_shape()
    image1 = resize(rgb, shape[1], shape[0])       
    assert_allclose(image1.shape[:2], shape)

    
    chars = "abcdsefghilmnopqrst"
    char2int = dict([(c, i) for i, c in enumerate(chars)])
    plan = tuple(map(char2int.__getitem__, planstring))
    
    
    # predict the result
    y0 = UncertainImage(image1)
    y1 = discdds.predict(y0, plan)
    
    tc = TestCase(id_tc=tcname, id_discdds=id_discdds,
                  y0=y0, y1=y1, true_plan=plan)

    return tc
