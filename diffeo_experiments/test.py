import numpy as np
from numpy import *
from PIL import Image
import pdb

import boot_agents
from boot_agents import *
#from diffeo import *



im = Image.open('/home/adam/bild.jpg')
im = im.resize((12,12))

img = np.array(im.getdata()).reshape(12,12)
print 'Hey'
#pdb.set_trace()
d = np.zeros((12,12,2))

for x in range(11):
	for y in range(11):
		d[x,y] = array([x+1,y])
v = np.zeros((12,12))
print d

D = Diffeomorphism2D(d,v)
print D

#img = np.random.random_integers(5,size=(8,8))/5
print img
img2 = diffeo_apply(d, img)
img2 = diffeo_apply(d, img2)
img2 = diffeo_apply(d, img2)
img2 = diffeo_apply(d, img2)
print img2



