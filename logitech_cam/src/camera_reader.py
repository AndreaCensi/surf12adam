#!/usr/bin/env python
import roslib 
import sys
roslib.load_manifest('logitech_cam')
import rospy
import random
from std_msgs.msg import String
import sensor_msgs
from sensor_msgs.msg import Image
import ParseMessages
import numpy
from Tkinter import *
from PIL import Image, ImageTk
import ImageChops, ImageOps

class camera_reader:
    def __init__(self,args):
        rospy.init_node('camera_reader', anonymous=True)

        try:
            inputstream = args[args.index('-s')+1]
        except ValueError:
            print 'listening to /usb_cam/image_raw'
            inputstream =  '/usb_cam/image_raw'
       
        subscriber = rospy.Subscriber(inputstream,sensor_msgs.msg.Image,self.callback)
        self.publisher = rospy.Publisher('/camera_reader/image',sensor_msgs.msg.Image)
        
        # Initiate a window
#        im = Image.open('/home/adam/bild.jpg')
#        im = im.resize((256,265))
#        print im
#        
#        self.window = Tk()
#        
#        tkim = ImageTk.PhotoImage(im)
#        print tkim
#        
#        self.w = Label(self.window, image=tkim)
#        print self.w
#        self.w.pack()
#        self.window.mainloop()
#        
#        
        # Window initiated
        
        rospy.spin()

        
    def callback(self, msg):
        self.publisher.publish(msg)
#        # For manipulating the images in array format
#        im,data,dimsizes = ParseMessages.imgmsg_to_pil(msg)
#        pix = numpy.asarray(im)
#        #pix = cmy_to_rgb(pix)
#        rosimg2 = ParseMessages.numpy_to_imgmsg(pix)
#        self.publisher.publish(rosimg2)
        
def cmy_to_rgb(data):
    #print dir(data)
    print data.shape
    data_out = numpy.zeros(data.shape,dtype=numpy.uint8)
    #print data_out.shape
#    Yp = 0.299 * data[:,:,0] + 0.114 * data[:,:,1] + 0.587 * data[:,:,2]
#    U = 0.492*(data[:,:,2] - Yp)
#    V = 0.877*(data[:,:,0] - Yp)
#    R = Yp + 1.13983*V
#    G = Yp + 
#    data_out[:,:,0] = (data[:,:,0] + 1.13983*data[:,:,2])
#    data_out[:,:,1] = (data[:,:,0] - 0.39465*data[:,:,1] - 0.58060 * data[:,:,2])
#    data_out[:,:,2] = (data[:,:,0] + 2.03211*data[:,:,1])
    data_out[:,:,0] = (1.0000*data[:,:,0] + 0.0000*data[:,:,1] + 0.0000*data[:,:,2])
    data_out[:,:,1] = (0.0000*data[:,:,0] + 1.0000*data[:,:,1] + 0.0000*data[:,:,2])
    data_out[:,:,2] = (0.0000*data[:,:,0] + 0.0000*data[:,:,1] + 1.0000*data[:,:,2])
#    data_out[:,:,0] = (data[:,:,1] + data[:,:,2])/2
#    data_out[:,:,1] = (data[:,:,0] + data[:,:,2])/2
#    data_out[:,:,2] = (data[:,:,0] + data[:,:,1])/2
    return data_out
        

def main(args):
    reader = camera_reader(args)
    print 'Shutting down'

if __name__ == '__main__':
    main(sys.argv)
