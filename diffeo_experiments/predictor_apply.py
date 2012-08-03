from PIL import Image
import numpy as np
from numpy import linalg as LA
import pdb
import sys
import pickle

def get_Y_pair((x0,y0),(dx,dy),(w,h),im):
	imc = im.crop((x0,y0,x0+w,y0+h))
	Y0 = np.array(imc.getdata(),np.uint8).reshape((h,w,3))

	imc2 = im.crop((x0+dx,y0+dy,x0+dx+w,y0+dy+h))
	Y1 = np.array(imc2.getdata(),np.uint8).reshape((h,w,3))
	return (Y0,Y1)	


def main(argv):
	im_test = Image.open('lighttower640.jpg')
	Y0,Y1 = get_Y_pair((300,300),(0,0),(160,120),im_test)







if __name__ == '__main__':
	main(sys.argv)
