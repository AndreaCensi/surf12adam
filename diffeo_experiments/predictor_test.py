from PIL import Image
import numpy as np
from numpy import linalg as LA
import pdb
import sys
#class predictor:
#	def __init__(self):

def compare(y0, yn, size):
	"""
		Compare reduced size of images. 
		Returns norm of difference in certain areas, 
		which is defined by the variance is less than 
		the average variance value. 
	"""
	y0_s = np.asarray(Image.fromarray(y0).resize(size)).astype(np.int16)
	yn_s = np.asarray(Image.fromarray(yn).resize(size)).astype(np.int16)
	diff = yn_s - y0_s
	norm = LA.norm(abs(diff))/diff.size
	mean = np.mean(diff)
	std = np.std(diff)
	return (mean, std, norm)

def get_Y_pair((x0,y0),(dx,dy),(w,h),im):
	imc = im.crop((x0,y0,x0+w,y0+h))
	Y0 = np.array(imc.getdata(),np.uint8).reshape((h,w,3))

	imc2 = im.crop((x0+dx,y0+dy,x0+dx+w,y0+dy+h))
	Y1 = np.array(imc2.getdata(),np.uint8).reshape((h,w,3))
	return (Y0,Y1)	

def main(argv):
	w = 160
	h = 120
#	pred = predictor()
#	im_test = Image.open('lighttower640.jpg')
	im_test = Image.open('randomimg.png')
#	im_test.show()
	sublist = np.array([[5,0],[-5,0],[0,5],[0,-5],[4,3],[3,4],[-3,4],[-4,3],[-4,-3],[-3,-4],[3,-4],[4,-3]])
	D_list = [[[0,0]],[[1,0],[-1,0],[0,1],[0,-1]],[[2,0],[-2,0],[0,2],[0,-2]],sublist,sublist*np.array([2]),sublist*np.array([5])]
	#dx_list = [0,1,1,1,1,1,2,2,2,2,3,4,5,5,5,5,6,7,8,9,10,10,10,10,20,30,40,50,60,90,120,160,200]
	for l in D_list:
		mean_l = []
		std_l = []
		norm_l = []
		t_kvantil = 1.72	# Depends on num iterations, n = 20
		#t_kvantil = 1.96
		for i in range(20):
			d = l[np.random.randint(0,len(l))]
			x0 = np.random.randint(0,im_test.size[0]-w)
			y0 = np.random.randint(0,im_test.size[0]-h)

			Y0,Y1 = get_Y_pair((x0,y0),d,(w,h),im_test)
			sim = compare(Y0,Y1,(8,6))
			mean_l.append(sim[0])
			std_l.append(sim[1])
			norm_l.append(sim[2])

		mean_mu = np.mean(mean_l)
		std_mu = np.std(mean_l)/np.sqrt(len(mean_l))
		mean_sigma = np.mean(std_l)
		std_sigma = np.std(std_l)/np.sqrt(len(std_l))
		mean_n = np.mean(norm_l)
		std_n = np.std(norm_l)/np.sqrt(len(norm_l))
		K_mu = [mean_mu-t_kvantil*std_mu, mean_mu+t_kvantil*std_mu]
		K_sigma = [mean_sigma-t_kvantil*std_sigma, mean_sigma+t_kvantil*std_sigma]
		K_n = [mean_n-t_kvantil*std_n, mean_n+t_kvantil*std_n]
		#print 'For d ~:',l[0],'I_mu = 	',K_mu,'	I_sigma = 	',K_sigma,
		print 'I_n = 	',K_n

	pdb.set_trace()
	
if __name__ == '__main__':
	main(sys.argv)
