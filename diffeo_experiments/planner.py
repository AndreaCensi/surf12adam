from diffeoplan.library.discdds import diffeo_action
from diffeoplan.library.images import uncertain_image
import pickle
import pdb
import numpy as np
from PIL import Image

diffeo_list = pickle.load(open('/media/data/learned_diffeo/diffeoptz1800w160.dynamics.pickle'))

action = diffeo_action.DiffeoAction('Test diffeo action object',
                                    diffeo_list[0],
                                    diffeo_list[1],
                                    np.array([0,150,0]))

im = Image.open('lighttower160.png')
#pdb.set_trace()
Y0 = uncertain_image.UncertainImage(np.array(im.getdata(),np.uint8).reshape((im.size[1],im.size[0],3)))

pdb.set_trace()