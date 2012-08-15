from . import logger
from ..diffeo_learner.ros_conversions import imgmsg_to_pil, pil_to_imgmsg
import rosbag
#from PIL import Image, ImageOps #@UnresolvedImport


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
        
        # Define some variables
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
        self.i = 0
        topics = ['/usb_cam/image_raw', '/logitech_cam/camera_executed']
        for topic, msg, t in self.bag.read_messages(topics=topics):
            if topic == '/logitech_cam/camera_executed':
                next_state = self.command_executed(msg, t)
                
            if topic == '/usb_cam/image_raw':
                next_state = self.cam_image_read(msg, t)
                
            self.state_capture = next_state

    def find_dt_threshold(self):
        '''
        Finds the value of dy/dt for when the robot is probably moving
        :param infile:    File to read data from
        '''
        logger.info('Calibrating diff_threshold for camera motion...')
        diffsum = 0
        numframes = 0
        ic = 0

        for _, msg, _ in self.bag.read_messages('/usb_cam/image_raw'):
            if ic > 300:
                break
            if ic < 100:
                diffsum = 0
            else:
                if self.Y_last != 0:
                    diff = compare_images(self.Y_last, msg)
                    diffsum += diff
                    numframes += 1
            self.Y_last = msg #@UnusedVariable
            ic += 1 
            
        diff_threshold = diffsum / numframes * 2
        logger.info('diff_threshold = %g ' % diff_threshold)
        return diff_threshold

    def add_container(self, Y_new):
        """
            Store the last nc images.
            To enable use of a image in the past when writing 
            Y0 since some motions may be to fast for capturing 
            the image after the command is executed.
        """ 
        for i in range(self.nc - 1):
            self.last_container[self.nc - i - 1] = self.last_container[self.nc - i - 2]
        self.last_container[0] = Y_new
        
    def command_executed(self, msg, t):
        if self.last_container[self.nc - 1] != 0:
            logger.debug('Camera Instruction Read: (%s)' % str(msg.data)) 
            logger.debug('Writing Y0 and U0')
            logger.debug('Time = %s' % t.to_time())
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
        logger.debug('cam_image_read()')
        next_state = self.state_capture
        if self.state_capture == PreProcessor.state_wait_for_move:
            diff = compare_images(self.Y_last, msg)
            logger.debug('Waiting for move state, diff = %g' % diff)
            if diff > self.diff_treshold:
                logger.debug('Camera detected to be moving')
                next_state = PreProcessor.state_cam_moving
            self.move_waits += 1

            if t.to_time() - self.t0.to_time() > self.move_time_timeout:
                logger.debug('Proceding to state_take_Y1, timeout in state_wait_for_move')
                next_state = PreProcessor.state_takeY1
                
        if self.state_capture == PreProcessor.state_cam_moving:
            diff = compare_images(self.Y_last, msg)
            logger.debug('Camera Moving State, diff = %s' % diff)
            if diff < self.diff_treshold:
                logger.debug('camera has stopped')
                interval = t.to_time() - self.t0.to_time()
                logger.debug('Motion took : %.3fs.' % interval)
                next_state = PreProcessor.state_takeY1
            self.move_waits += 1

            if t.to_time() - self.t0.to_time() > self.move_time_timeout:
                logger.debug('Proceding to state_take_Y1, timeout in state_cam_moving')
                next_state = PreProcessor.state_takeY1
                
        if self.state_capture == PreProcessor.state_takeY1:
            logger.debug('Writing Y1')
            write_img = self.zoom_image(msg, self.zoom)
            self.out_bag.write('Y1', write_img, self. t0)
            self.i += 1
            
            next_state = PreProcessor.state_takeY0
        
        self.add_container(msg)

        self.Y_last = msg    # update last read image
        self.zoom_last = self.zoom
        
        return next_state
    
    def zoom_image(self, image, zoom):
        """ Zoom an image with zoom center. """
        pim, _, (h, w, _) = imgmsg_to_pil(image)
        z = float(zoom)
        z0 = 100.0 # original size zoom

        x0 = int(w / 2 * (1.0 - z0 / z)) 
        y0 = int(h / 2 * (1.0 - z0 / z)) 
        dx = int(w / 2 * (1.0 + z0 / z)) 
        dy = int(h / 2 * (1.0 + z0 / z)) 
        pim_crop = pim.crop((x0, y0, dx, dy))
        pim_out = pim_crop.resize(self.output_size)    
        return pil_to_imgmsg(pim_out)


def compare_images(image1, image2, step=100):
    """
     Compares two images (/sensor_msgs/Image) and estimates first norm 
     of the difference between the pixel values by calculating the 
     average difference for the set of each <step>'th pixel.
     
     Output
         Average pixel value difference
    """
    normdiff = 0.
    n = len(image1.data)
    for i in range(0, n - 1, step):
        a = int(image1.data[i].encode('hex'), 16) 
        b = int(image2.data[i].encode('hex'), 16)
        normdiff += abs(a - b)    
    return normdiff / n / step 

