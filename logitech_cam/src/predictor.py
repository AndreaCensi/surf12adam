#!/usr/bin/env python
import roslib 
import rosbag
import sys
roslib.load_manifest('logitech_cam')
import rospy
from std_msgs.msg import String
import sensor_msgs
from sensor_msgs.msg import Image
import ParseMessages
import numpy
from numpy import linalg as LA
from PIL import Image
import ImageChops, ImageOps
import boot_agents
from boot_agents import *
import pdb
import pickle
from learner import learner

class predictor:
    """
        
    """
    def apply(self, y0, index):
        """
            Apply diffeomorphism number <index> to image <y0> 
        """
        
    def compare(self, y0, yn, size):
        """
            Compare reduced size of images. 
            Returns norm of difference in certain areas, 
            which is defined by the variance is less than 
            the average variance value. 
        """
    def find_neighbor_indices(self, y0):
        #pdb.set_trace()
        self.estimator = learner((y0.shape[1],y0.shape[0]),[4,4]).new_estimator()
        self.estimator.init_structures(y0[:,:,0])
        #self.neighbor_indices = estimator.neighbor_indices
        
    def compare_neighbor(self, y0, yn, var=NONE):
        """
            Compare fullsize of images in neighbour area
        """
        
        estr = learner((y0.shape[1],y0.shape[0]),[4,4]).new_estimator()
        estr.update(y0[:,:,0],yn[:,:,0])
        estr.update(y0[:,:,1],yn[:,:,1])
        estr.update(y0[:,:,2],yn[:,:,2])
        n = len(estr.neighbor_similarity_flat)
        best_sim = np.array([[0]]*n)
        for k in range(n):
            best_sim[k] = max(estr.neighbor_similarity_flat[k])
        if var != NONE:
            best_sim = best_sim*var.flat
        return np.mean(best_sim)
        
        
    def apply_random_sequence(self, y0, level=5):
        """
            Apply diffeomorphisms on image <y0> in a 
            randomized sequence of depth <level>. 
            Returns the command sequence as a <level>-tuple 
            and the final image.
        """
        
    def search_for_plan(self,y0,yn,nitr=10,level=5):
        """
            Search for a sequence of diffeomorphisms that 
            transforms <y0> to <yn>.
            Max <nitr> random sequences is tried, max length 
            of sequences is <level>
        """
        size = [16,12] 
        for i in range(nitr):
            # get a estimated final image and the sequence to the final image
            y_est, seq = self.apply_random_sequence(yo, level)
            
            # compare the estimated image against the goal.
            sim = self.compare(y0, yn, size)
    def test_compare_neighbor(self,argv):
        def get_Y_pair((x0,y0),(dx,dy),(w,h),im):
            imc = im.crop((x0,y0,x0+w,y0+h))
            Y0 = np.array(imc.getdata(),np.uint8).reshape((h,w,3))
        
            imc2 = im.crop((x0+dx,y0+dy,x0+dx+w,y0+dy+h))
            Y1 = np.array(imc2.getdata(),np.uint8).reshape((h,w,3))
            return (Y0,Y1)
        
        # Find Input image
        try:
            infile = argv[argv.index('-im')+1]
        except ValueError:
            infile = '/home/adam/git/surf12adam/diffeo_experiments/lighttower640.jpg'
        print 'Using image file:                ',infile
        
        
        im = Image.open(infile)
        print 'Compare two images close to each other'
        y0, y1 = get_Y_pair((260,300),(2,0),(160,120),im)
        self.find_neighbor_indices(y0)
        print self.compare_neighbor(y0, y1)
        
        print 'Compare two images far from each other'
        y0, y1 = get_Y_pair((260,300),(5,0),(160,120),im)
        print self.compare_neighbor(y0, y1)
        
        print 'Compare two images really far from each other'
        y0, y1 = get_Y_pair((260,300),(200,0),(160,120),im)
        print self.compare_neighbor(y0, y1)
        
        print 'Compare two images close to each other with nois'
        y0, y1 = get_Y_pair((260,300),(0,0),(160,120),im)
        #pdb.set_trace()
        y1 = y1 + np.random.randint(0,10,y0.shape)
        y1 = (y1*255.0/np.max(y1)).astype(np.uint8)
#        pdb.set_trace()
        print self.compare_neighbor(y0, y1)
        
        
def main(argv):
    predict = predictor()
    predict.test_compare_neighbor(argv)
#    predict.compare_neighbor(y0, yn)
    
if __name__ == '__main__':
    main(sys.argv)