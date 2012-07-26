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
from PIL import Image
import ImageChops, ImageOps
import boot_agents
from boot_agents import *
import pdb
import pickle

def get_image_array(image):
    im,data,dimsizes = ParseMessages.imgmsg_to_pil(image)
    pix = numpy.asarray(im).astype(numpy.uint8)
    return pix

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

class learner:
    ''' Organizes a list of diffeomorphism estimators to learn the diffeomorphisms '''
    def new_estimator(self):
        width = self.size[0]
        height = self.size[1]
        s_w = float(self.search_area[0])
        s_h = float(self.search_area[1])
        return DiffeomorphismEstimator((s_h/height,s_w/width),"continuous")
    
    def command_index(self,command):
        try: 
            index = self.command_list.index(command)
        except ValueError:
            print 'Adding new command to command_list: ',str(command)
            self.command_list.append(command)
            self.estimators.append(self.new_estimator())
            index =  self.command_list.index(command)
        return index
    def update_ros(self,Y0_ros,U0_ros,Y1_ros):
        Y0 = get_image_array(Y0_ros)
        Y1 = get_image_array(Y1_ros)
        U0 = U0_ros.data
        self.update(Y0, U0, Y1)
        
    def update(self,Y0,U0,Y1):
        cmd_ind = self.command_index(U0)
        print 'Uppdating estimator ',str(cmd_ind)
        #pdb.set_trace()
        for ch in range(3):
            self.estimators[cmd_ind].update(Y0[:,:,ch],Y1[:,:,ch])
        
    def learn_bag(self,bagfile):
        print 'Learning from: ',bagfile
        bag = rosbag.Bag(bagfile)
        list = bag.read_messages(topics=['Y0','Y1','U0'])
        
        i = 0
        r = 0
        for topic, msg, t in list:
            print 'Reading cmd'
            if topic == 'Y0':
                print 'Y0 read @',t
                Y0_ros = msg
                if r == 0:
                    r = 1
                    t0 = t
            if topic == 'U0':
                print 'U0 read @',t
                U0_ros = msg
                if r <> 0:
                    if t0 == t:
                        r += 1
                    else:
                        print 'Error interpreting bag, unexpected timestamp, ignoring message.'
                        r = 0
                    
            if topic == 'Y1':
                print 'Y1 read @',t
                Y1_ros = msg
                if r <> 0:
                    if t0 == t:
                        r += 1
                    else:
                        print 'Error interpreting bag, unexpected timestamp, ignoring message.'
                        r = 0
            # If r == 3, then a complete image pair and command is read
            if r == 3:
                print 'Updating estimators ',i
                i += 1
                self.update_ros(Y0_ros, U0_ros, Y1_ros)
                r = 0
                            
    def summarize(self):
        """
            Summarizes all estimators
            Output:
                All summarized diffeomorphisms stored in self.diffeo_list
        """
        n = len(self.estimators)
        self.diffeo_list = [[]]*n
        for i in range(n):
            self.diffeo_list[i] = self.estimators[i].summarize()
                
    def diffeo_dump(self,file):
        ''' Save all summarized diffeomorphisms to a pickle file '''
        for i in range(len(self.diffeo_list)):
            self.diffeo_list[i].command = self.command_list[i]
        pickle.dump(self.diffeo_list,file)
            
    def show_diffeomorphisms(self):
        for i in range(len(self.estimators)): #estr in self.estimators:
            D = self.estimators[i].summarize()
            cmd = self.command_list[i]
            save_path = '/home/adam/'
            #Image.fromarray(diffeo_to_rgb_angle(D.d)).show()
            Image.fromarray(diffeo_to_rgb_angle(D.d)).save(save_path+'cmd'+str(cmd).replace(' ','')+'angle.png')
            Image.fromarray(diffeo_to_rgb_norm(D.d)).save(save_path+'cmd'+str(cmd).replace(' ','')+'nomr.png')
            Image.fromarray((D.variance*255).astype(np.uint8)).save(save_path+'cmd'+str(cmd).replace(' ','')+'variance.png')
            #Image.fromarray(diffeo_to_rgb_angle(D.d)).save('diffeoimages/dir'+str(delta[0])+','+str(delta[1])+'_n'+str(n)+'_image_'+image_str+'_phase.png')
        #pdb.set_trace()
        
        
        
        
    def __init__(self,size,search_area):
        print 'Learner Initializing...'
        self.command_list = []
#        self.diffeomorphisms = []
        self.estimators = []
        
        self.size = size
        self.search_area = search_area
        
def test_diffeo(argv):
    # Find diffeomorphisms file
    try:
        dfile = argv[argv.index('-dl')+1]
    except ValueError:
        dfile = '/media/data/learned_diffeo/camera_ptz.dynamics.pickle'
    print 'Using diffeomorphism from file:    ',dfile
    
    # Find diffeomorphisms file
    try:
        outpath = argv[argv.index('-o')+1]
    except ValueError:
        outpath = '/media/data/tempimages/'
    print 'Saving images to path:            ',outpath
    
    # Find output prefix file
    try:
        prefix = argv[argv.index('-p')+1]
    except ValueError:
        prefix = 'logitech_cam_ptz'
    print 'Using prefix for output files:    ',prefix
    
    # Find Input image
    try:
        infile = argv[argv.index('-im')+1]
    except ValueError:
        infile = '/home/adam/git/surf12adam/diffeo_experiments/lighttower640.jpg'
    print 'Using image file:                ',infile
    
    # Number of levels to apply
    try:
        levels = int(argv[argv.index('-l')+1])
    except ValueError:
        levels = 5
    print 'Applying diffeomorphism ',levels,' times'

    # Image size
    try:
        size = eval(argv[argv.index('-size')+1])
    except ValueError:
        size = [160,120]
    print 'Image size:    ',size
    
    im = np.array(Image.open(infile).resize(size).getdata(),np.uint8).reshape((size[1],size[0],3))
    #Y0,Y1 = get_Y_pair((30,30),(0,0),(160,120),im)
    pdb.set_trace()
    diffeo_list = pickle.load(open(dfile,'rb'))
    for D in diffeo_list:
        Y = im
        Image.fromarray(Y).save(outpath+prefix+str(D.command)+str(0)+'.png')
        for i in range(levels):
            Y, var = D.apply(Y)
            Image.fromarray(Y).save(outpath+prefix+str(D.command)+str(i+1)+'.png')

def main(args):
    print args
    try:
        bagfile = args[args.index('-i')+1]
    except ValueError:
        print 'No input bag specified'
    try:
        dfile = args[args.index('-o')+1]
    except ValueError:
        print 'No input bag specified'
    try:
        size = eval(args[args.index('-size')+1])
    except ValueError:
        size = [160,120]
    try:
        area = eval(args[args.index('-area')+1])
    except ValueError:
        area = [8,8]
#    try:
#        w = args[args.index['-w']+1]
#    except ValueError:
#        print 'No input width specified'
#    try:
#        h = args[args.index['-h']+1]
#    except ValueError:
#        print 'No input height specified'
    pdb.set_trace()
    learn = learner(size,area)
    learn.learn_bag(bagfile)
    print 'Commands: ',learn.command_list
    learn.summarize()
    #pdb.set_trace()
    learn.diffeo_dump(open(dfile,'wb'))
    learn.show_diffeomorphisms()
#    print learn.command_index([0,0,0])
#    print learn.command_index([1,0,0])
#    print learn.command_index([1,0,0])
#
#if __name__ == '__main__':
#    main(sys.argv)





if __name__ == '__main__':
    try:
        if sys.argv.index('test'):
            test = True
            learn = False
        else:
            test = False
            learn = True
    except ValueError:
        test = False
        learn = True
    if learn:
        main(sys.argv)
    if test:
        test_diffeo(sys.argv)