import numpy as np
from numpy import *
from PIL import Image, ImageOps, ImageChops
import boot_agents
from boot_agents import *
import time
import pdb
import diffeo_demo
import sys
from sys import stdout

w = 160
h = 120
# Returns the image with upper left corner at (x0,y0) with width and height w and h, and the image with corner at (x0+dx,y0+dy)
def get_Y_pair((x0,y0),(dx,dy),im):
	imc = im.crop((x0,y0,x0+w,y0+h))
	Y0 = np.array(imc.getdata(),np.uint8).reshape((h,w,3))

	imc2 = im.crop((x0+dx,y0+dy,x0+dx+w,y0+dy+h))
	Y1 = np.array(imc2.getdata(),np.uint8).reshape((h,w,3))
	return (Y0,Y1)

def get_Y_pair_random(size,delta,im):
	return get_Y_pair((np.random.randint(abs(delta[0]),im.size[0]-abs(delta[0])),np.random.randint(abs(delta[1]),im.size[1]-abs(delta[1]))),delta,im)
	
def add_noise(Y, noise):
	"""
	Add a noise of max size <noise> to the image Y
	"""
	dtype0 = Y.dtype
	N = np.random.randint(-noise,noise,Y.shape).astype(np.int16)
	Y = N + Y
	if dtype0 == np.uint8:
		Y[Y<0]=0
		Y[Y>255]=255
	return Y.astype(dtype0)
	

def get_angle(D):
	Y,X = np.meshgrid(range(y0.shape[1]),range(y0.shape[0]))
	Dx = D.d[:,:,0]-X
	Dy = D.d[:,:,1]-Y
	#print Dx
	#print Dy
	Dx_f = Dx.reshape(Dx.size)
	Dy_f = Dy.reshape(Dy.size)
	var_f = D.variance.reshape(Dx.size)
	var_mean = mean(var_f)
	at = []
	for i in range(Dx.size):
		if abs(var_f[i]) > abs(var_mean):
			at.append(np.arctan2(Dy_f[i],Dx_f[i]))
	#at = np.arctan(D.d[:,:,1]-Y,D.d[:,:,0]-X)
	return (np.mean(at), np.std(at))

if __name__ == '__main__':
	if '--help' in sys.argv:
		print('Learn_demo')
		print('')
		print('		-i		input image name, without extension which has to be .png')
		print('				enter \'random\' to generate a random image ')
		print('		-nl		nois level on Y1')
		print('		-delta	offset between images')
		print('		-search	size of area to search, area is searchxsearch')
		print('		-size	size of images to analyze')
		print('')
		print('')
		
	print 'starting main'
	print sys.argv


	#print sys.argv[sys.argv.index('-d')+1].split(',')
	#print [int(s) for s in sys.argv[sys.argv.index('-d')+1].split(',')]
	try:
		#delta = [int(s) for s in sys.argv[sys.argv.index('-d')+1].split(',')]
		delta = eval(sys.argv[sys.argv.index('-delta')+1])
	except ValueError:
		delta = [2,0]
	print 'delta', delta
	try:
		n = int(sys.argv[sys.argv.index('-n')+1])
	except ValueError:
		n = 10
	print 'n: ', n

	# noise level
	try:
		nl = float(sys.argv[sys.argv.index('-nl')+1])
	except ValueError:
		nl = 0

	# Search Area = sa*sa
	try:
		sa = float(sys.argv[sys.argv.index('-search')+1])
	except ValueError:
		sa = 5.0
		
	# Image size
	try:
		size = float(sys.argv[sys.argv.index('-size')+1])
	except ValueError:
		size = [160,120]
		
	# Input image str
	try:
		image_str = sys.argv[sys.argv.index('-i')+1]
	except ValueError:
		image_str = 'random'
	print image_str
	
	# Load image to crop subimages from
	if image_str == 'random':
		M = np.random.randint(0,255,(748,1024,3)).astype(np.uint8)
		im = Image.fromarray(M)
	else:
		im = Image.open(image_str+'.png')
		#im = im.resize((2048/8,1536/8))
	

	Y0,Y1 = get_Y_pair_random(size,delta,im)
	Y1 = add_noise(Y1,nl)

	# if argument demoseq, show image and two sub images
	if 'demoseq' in sys.argv:
		Image.fromarray(Y0).show()
		Image.fromarray(Y1).show()

#	pdb.set_trace()
	print 'Initiate learning'
	estr = DiffeomorphismEstimator((sa/h,sa/w),"continuous")
	# Learn from n images times 3 channels
	for i in range(n):
		if i%(n/10) == 0:
			print float(i)/float(n)*100, '% Completed'
		for ch in range(3):
			y0 = Y0[:,:,ch]
			y1 = Y1[:,:,ch]
			estr.update(y0,y1)
		Y0,Y1 = get_Y_pair_random(size,delta,im)
		Y1 = add_noise(Y1,nl)
		#Y0,Y1 = get_Y_pair((np.random.randint(abs(delta[0]),im.size[0]-abs(delta[0])),np.random.randint(abs(delta[1]),im.size[1]-abs(delta[1]))),delta,im)
	print 'summarizing'
	D = estr.summarize_smooth()
	# Save diffeomorphism images
	Image.fromarray(diffeo_to_rgb_norm(D.d)).save('diffeoimages/dir'+str(delta[0])+','+str(delta[1])+'_n'+str(n)+'_image_'+image_str+'_modulus.png')
	Image.fromarray(diffeo_to_rgb_angle(D.d)).save('diffeoimages/dir'+str(delta[0])+','+str(delta[1])+'_n'+str(n)+'_image_'+image_str+'_phase.png')
#	pdb.set_trace()
	Image.fromarray((D.variance*255).astype(np.uint8)).save('diffeoimages/dir'+str(delta[0])+','+str(delta[1])+'_n'+str(n)+'_image_'+image_str+'_variance.png')

	# Print the learned angle
	(ang_m, ang_s) = get_angle(D)
	print 'Correct angle is: ',np.arctan2(delta[0],delta[1])
	print 'Learned angle is: ',ang_m,' +- ',ang_s
	
	print 'diffeo_estimator_demo done'
	#pdb.set_trace()

