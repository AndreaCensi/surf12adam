#!/usr/bin/env python
from PIL import Image, ImageOps #@UnresolvedImport @UnusedImport
from learner.programs.diffeo_learner.ros_conversions import * #@UnusedWildImport
from optparse import OptionParser
import rosbag
roslib.load_manifest('logitech_cam')


#def resize_image(image, size):
#    # Convert image to cv image to zoom
#    try:
#        cv_image = bridge.imgmsg_to_cv(image, "bgr8")
#    except CvBridgeError, e:
#        print e
#    cvi2 = cv.CreateImage(size, cv.IPL_DEPTH_8U, 3)
#    cv.Resize(cv_image, cvi2, interpolation=cv.CV_INTER_LINEAR)
#    return bridge.cv_to_imgmsg(cvi2, "bgr8")




# compare_images
#
# Compares two images (/sensor_msgs/Image) and estimates first norm 
# of the difference between the pixel values by calculating the 
# average difference for the set of each <step>'th pixel.
# 
# Output
#     Average pixel value difference
#   
def compare_images(image1, image2, step=100):
    normdiff = 0.
    n = len(image1.data)
    for i in range(0, n - 1, step):
        normdiff += abs(int(image1.data[i].encode('hex'), 16) - int(image2.data[i].encode('hex'), 16))    
    return normdiff / n / step 
   
   
## returns the value of a pixel at position x,y in image image 
#def get_pixel(image, x, y): # BUG!
#    width = image.width
#    height = image.height
#    pix_index = y * width + height 
#    return int(image.data[pix_index].encode('hex'), 16)
   
       
#def get_area_average(image, x, y, w, h):
#    width = image.width
#    height = image.height
#    sum = 0
#    for r in range(h - 1):
#        for c in range(w - 1):
#            sum += get_pixel(image, x + c, y + r)
#    sum = sum / w / h # bug: division by integer (w, h is integer: 1/320 = 0)
#    return sum
    

class PreProcessor():
    # Define state integers for image capturing states
    state_takeY0 = 0            # Waiting for first image
    state_wait_for_move = 1        # Wait for camera to start moving
    state_cam_moving = 2        # State for when camera is moving
    state_takeY1 = 3            # Waiting for last image
    
    def __init__(self, infile, outfile, output_size, nc=10):
        self.nc = nc # Number of history images to store in array
        self.last_container = [0] * nc
        self.output_size = output_size
        self.Y_last = 0
        
        # Keep track of the actual zoom for the last command and the current zoom
        self.zoom_last = 250
        self.zoom = 100
        
        
        self.bag = rosbag.Bag(infile)
        self.out_bag = rosbag.Bag(outfile, 'w')
        
        
        
        # Define some parameters
        self.next_state = -1
        self.state_capture = 0
        self.diff_treshold = self.find_dt_threshold()

        self.move_waits = 0
        self.move_timeout = 9
        self.move_time_timeout = 0.5
        
    def finalize(self):
        self.bag.close()
        self.out_bag.close()
        
    def process_bag(self):
#        data_list = 
        self.i = 0
        for topic, msg, t in self.bag.read_messages(topics=['/usb_cam/image_raw', '/logitech_cam/camera_executed']):
            #pdb.set_trace()
            if topic == '/logitech_cam/camera_executed':
                #if Y_last != 0:
                next_state = self.command_executed(msg, t)
                
            if topic == '/usb_cam/image_raw':
                #print 'cam image read'
                next_state = self.cam_image_read(msg, t)
                
            self.state_capture = next_state

    def find_dt_threshold(self):
        '''
        Finds the value of dy/dt for when the robot is probably moving
        :param infile:    File to read data from
        '''
        print('Calibrating diff_threshold for camera motion...')
#        imglist = 
        diffsum = 0
        numframes = 0
        ic = 0
#        for topic, msg, t in imglist:
        for _, msg, _ in self.bag.read_messages('/usb_cam/image_raw'):
#            print('.')
            if ic > 300:
                break
            if ic < 100:
                diffsum = 0
            else:
                if self.Y_last != 0:
                    diff = compare_images(self.Y_last, msg)
#                    print diff
                    diffsum += diff
                    numframes += 1
            self.Y_last = msg #@UnusedVariable
            ic += 1 
#        print ic
        diff_threshold = diffsum / numframes * 2
        print 'diff_threshold = ', diff_threshold
        return diff_threshold

    def add_container(self, Y_new):
        # Store the last nc images.
        # To enable use of a image in the past when writing 
        # Y0 since some motions may be to fast for capturing 
        # the image after the command is executed. 
        for i in range(self.nc - 1):
            self.last_container[self.nc - i - 1] = self.last_container[self.nc - i - 2]
        self.last_container[0] = Y_new
        
        

    def command_executed(self, msg, t):
        if self.last_container[self.nc - 1] != 0:
            print 'Camera Instruction Read: (', msg.data, ')' 
            print 'Writing Y0 and U0'
            print 'Time = ', t.to_time()
            write_img = self.zoom_image(self.last_container[5], self.zoom_last)
#            write_img = resize_image(zoom_image(self.last_container[5], self.zoom_last), self.output_size)
            self.out_bag.write('Y0', write_img, t)
            self.out_bag.write('U0', msg, t)
            
            # self.t0 is the time when the last command was executed/posted
            self.t0 = t
            
            if self.zoom_last + msg.data[2] >= 100:
                if self.zoom_last + msg.data[2] <= 500:
                    self.zoom = self.zoom_last + msg.data[2]
            next_state = PreProcessor.state_wait_for_move
            self.move_waits = 0
            
            return next_state
            
    def cam_image_read(self, msg, t):
        print('cam_image_read()')
        next_state = self.state_capture
        if self.state_capture == PreProcessor.state_wait_for_move:
            diff = compare_images(self.Y_last, msg)
            print 'Waiting for move state, diff = ',diff
            if diff > self.diff_treshold:
                print 'Camera detected to be moving'
                next_state = PreProcessor.state_cam_moving
            self.move_waits += 1
            #if move_waits > move_timeout:
            #pdb.set_trace()
            if t.to_time() - self.t0.to_time() > self.move_time_timeout:
                print 'Proceding to state_take_Y1, timeout in state_wait_for_move'
                next_state = PreProcessor.state_takeY1
                
        if self.state_capture == PreProcessor.state_cam_moving:
            diff = compare_images(self.Y_last, msg)
            print 'Camera Moving State, diff = ', diff
            if diff < self.diff_treshold:
                print 'camera has stoped'
                print 'Motion took :', t.to_time() - self.t0.to_time(), ' s'
                next_state = PreProcessor.state_takeY1
            self.move_waits += 1
            #if move_waits > move_timeout:
            if t.to_time() - self.t0.to_time() > self.move_time_timeout:
                print 'Proceding to state_take_Y1, timeout in state_cam_moving'
                next_state = PreProcessor.state_takeY1
                
        if self.state_capture == PreProcessor.state_takeY1:
            print 'Writing Y1'
            #print output_size
            #print output_size.__class__
            write_img = self.zoom_image(msg, self.zoom)
            self.out_bag.write('Y1', write_img,self. t0)
            self.i += 1
            
            next_state = PreProcessor.state_takeY0
        
        self.add_container(msg)

        self.Y_last = msg    # update last read image
        self.zoom_last = self.zoom
        
        return next_state
    # Zoom an image with zoom center
    def zoom_image(self,image, zoom):
        pim, _, (h, w, _) = imgmsg_to_pil(image)
        z = float(zoom)
        z0 = 100.0 # original size zoom
        
        x0 = int(w/2*(1.0 - z0/z))
        y0 = int(h/2*(1.0 - z0/z))
        dx = int(w*z0/z)
        dy = int(h*z0/z)
        pim_crop = pim.crop((x0, y0, dx, dy))
        pim_out = pim_crop.resize(self.output_size)    
        return pil_to_imgmsg(pim_out)

def main():
    usage = "usage: %prog -n name -i inputpath -o outputpath -s [W,H] -p string"
    parser = OptionParser(usage=usage, version="%prog 1.0")
    parser.add_option("-n", "--name", default='data',
                      help="Name of log without file extension, input has to be .raw.bag and output will be .processed.bag")
    parser.add_option("-i", "--input", default='/media/data/raw-capture/', help="Path to input file")
    parser.add_option("-o", "--output", default='/media/data/processed-data/', help="Path to output file")
    parser.add_option("-p", "--namefix", default='', help='Additional string for output filename')
    parser.add_option("-s", "--size", default='[160,120]', help="Image size WxH")
#    parser.add_option("-zc", "--zoom_center", default=[0,0], help="Not Impl: Zoom center offset from mid image")
    options, _ = parser.parse_args()
    infile = options.input + options.name + '.raw.bag'
    outfile = options.output + options.name + options.namefix + '.processed.bag'
    output_size = eval(options.size)
    
    pproc = PreProcessor(infile, outfile, output_size)
    pproc.process_bag()
    pproc.finalize()

if __name__ == '__main__':
    main()