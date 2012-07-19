import numpy as np
from numpy import *
from PIL import Image, ImageOps, ImageChops
import boot_agents
from boot_agents import *
import time
import pdb
import diffeo_demo
import sys

##
#	Troubleshouting
#	Don't forget to have scikit-image installed
#
def get_Y_pair((x0,y0),(dx,dy),im):
	w = 40
	h = 30
	imc = im.crop((x0,y0,x0+w,y0+h))
	#imc.show()
	Y0 = np.array(imc.getdata(),np.uint8).reshape((h,w,3))

	imc = im.crop((x0+dx,y0+dy,x0+dx+w,y0+dy+h))
	#imc.show()
	Y1 = np.array(imc.getdata(),np.uint8).reshape((h,w,3))
	#return (Image.fromarray(Y0), Image.fromarray(Y1))
	return (Y0,Y1)

def generate_from_croped_image():
	print sys.argv[sys.argv.index('-d')+1].split(',')
	print [int(s) for s in sys.argv[sys.argv.index('-d')+1].split(',')]
	try:
		delta = [int(s) for s in sys.argv[sys.argv.index('-d')+1].split(',')]
	except ValueError:
		delta = [20,0]
	print 'delta', delta
	try:
		n = int(sys.argv[sys.argv.index('-n')+1])
	except ValueError:
		n = 10
	print 'n: ', n

	# Load and resize image to crop subimages from
	#im = Image.open('balboa.png')
	#im = im.resize((2048/8,1536/8))
	
	# Generate random image to use
	M = np.random.randint(0,255,(748,1024,3))
	im = Image.fromarray(M.astype(np.uint8))
#	im.show()


#	pdb.set_trace()
	Y0,Y1 = get_Y_pair((5,20),delta,im)
#	Y0.show()
#	Y1.show()



	print 'Initiate learning'
	estr = DiffeomorphismEstimator((0.5,0.5),"continuous")

	for i in range(n-1):
		for ch in range(3):
			print 'Applying command: ',str(i),'.',str(ch)
			y0 = Y0[:,:,ch]
			y1 = Y1[:,:,ch]
			estr.update(y0,y1)
		Y0,Y1 = get_Y_pair((np.random.randint(abs(delta[0]),im.size[0]-abs(delta[0])),np.random.randint(abs(delta[1]),im.size[1]-abs(delta[1]))),delta,im)
	print 'summarizing'
	D = estr.summarize()
#	Image.fromarray(diffeo_to_rgb_inc(D.d)).show()
	Image.fromarray(diffeo_to_rgb_norm(D.d)).save('diffeoimages/diffeo_to_rgb_norm_delta_'+str(delta)+'_n_'+str(n)+'.png')
	Image.fromarray(diffeo_to_rgb_angle(D.d)).save('diffeoimages/diffeo_to_rgb_angle_delta_'+str(delta)+'_n_'+str(n)+'.png')
#	Image.fromarray(D.variance*255).show()#.save('diffeoimages/variance_delta_'+str(delta)+'_n_'+str(n)+'.png')
#	pdb.set_trace()
	print 'diffeo_estimator_demo done'

#
#
#
#
def ideal_diffeo_show():
	try:
		command_list = sys.argv[sys.argv.index('-c')+1]
	except ValueError:
		command_list = [2,2,2,2]
	print command_list
	# -d delta_x,delta_y
	

#	pdb.set_trace()
	print 'Generating demo diffeomorphisms'
	size = (120,160)
	diffeo_list = [diffeo_demo.diffeo_zoomin(size,0.9), diffeo_demo.diffeo_zoomin(size,1.1),diffeo_demo.diffeo_pan(size,0.1),diffeo_demo.diffeo_pan(size,-0.1),diffeo_demo.diffeo_tilt(size,0.1),diffeo_demo.diffeo_tilt(size,-0.1)]
	print 'Done'

	d_index = 2
#	Image.fromarray(diffeomorphism_to_rgb(diffeo_list[d_index])).show()
#	Image.fromarray(diffeomorphism_to_rgb_cont(diffeo_list[d_index])).show()
#	Image.fromarray(diffeo_to_rgb_norm(diffeo_list[d_index])).show()
	Image.fromarray(diffeo_to_rgb_angle(diffeo_list[d_index])).show()

	pdb.set_trace()
	# Generate a sequence of images to use for learning
	print 'Generating images'
	im = Image.open('lighttower160.jpg')

	im_array = np.array(im.getdata(),np.uint8).reshape((120,160,3))
	im_array_new = im_array
#	command_list = [2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2]
#	command_list = [4,4,4,4,4,4,4,4,4,4,4,4,4,4,4]
	n = len(command_list)
	images = [im_array]
	#print images
	print 'Applying diffeomorphisms: ',command_list
	for i in range(n):
		command = command_list[i]
		im_array_new = diffeo_apply(diffeo_list[command],im_array_new)
		images.append(im_array_new)
	print 'done'
	pdb.set_trace()


if __name__ == '__main__':
	print 'starting main'
	print sys.argv
	ideal_diffeo_show()
#	generate_from_croped_image()
#	pdb.set_trace()
