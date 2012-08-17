from . import logger
from ..diffeo_learner.ros_conversions import imgmsg_to_pil, pil_to_imgmsg
import rosbag
import numpy as np
import pdb
from learner.programs.diffeo_learner.bag_reader import get_image_array
#from PIL import Image, ImageOps #@UnresolvedImport

    
topic_image_raw = '/usb_cam/image_raw'
topic_camera_executed = '/logitech_cam/camera_executed'


class CameraTracker:
    
    def __init__(self, threshold):
        self.last_stopped = None
        self.last = None
        self.threshold = threshold
    
    def push(self, t, image):
        if self.last is None:
            # this is the first image
            # we consider it moving
            pass
        else:
            # we have some image to compare
            diff = image_distance(image, self.last)
            stopped = diff < self.threshold 
            if stopped:
                # this is a stopped image, so we remember it
                self.last_stopped = image
                self.last_stopped_t = t
            self.last_was_moving = not stopped
        self.last = image
        self.last_t = t
        
    def was_moving(self):
        """ Return true if the last image was moving. """
        assert self.last is not None
        return self.last_was_moving
    
    def has_last_stopped(self):
        return self.last_stopped is not None
    
    def get_last_stopped(self):
        assert self.has_last_stopped()
        return self.last_stopped


#state_takeY0 = 0            # Waiting for first image
#state_wait_for_move = 1        # Wait for camera to start moving
#state_cam_moving = 2        # State for when camera is moving
#state_takeY1 = 3            # Waiting for last image

STATE_START = 'START'
STATE_WAIT_CMD = 'WAIT_CMD'
STATE_WAIT_FOR_MOVING = 'WAIT_FOR_MOVING'
STATE_WAIT_FOR_STOP = 'WAIT_FOR_STOP'
EVENT_STOPPED = 'STOPPED'
EVENT_MOVING = 'MOVING'
EVENT_CMD = 'CMD'

class StateMachine:
                                
    def __init__(self):
        self.state2handler = {}
        self.state2handler[STATE_START] = self.handle_start
        self.state2handler[STATE_WAIT_CMD] = self.handle_wait_cmd
        self.state2handler[STATE_WAIT_FOR_MOVING] = self.handle_wait_for_moving
        self.state2handler[STATE_WAIT_FOR_STOP] = self.handle_wait_for_stop
        self.state = STATE_START   
        
        self.queue = [] 
        
    def dispatch(self, event, event_parameter):
        handler = self.state2handler[self.state]
        next_state = handler(event, event_parameter)
        if next_state is None:
            next_state = self.state
        assert next_state in self.state2handler
        
    def received_stopped_image(self, t, image):
        self.dispatch(EVENT_STOPPED, (t, image))
    
    def received_moving_image(self, t, image):
        self.dispatch(EVENT_MOVING, (t, image))
    
    def received_command(self, t, command):
        self.dispatch(EVENT_CMD, (t, command))
    
    
    def handle_start(self, event, params):
        if event == EVENT_STOPPED:
            t, image = params
            self.y0 = (t, image)
            return STATE_WAIT_CMD
        elif event == EVENT_MOVING:
            return  
        elif event == EVENT_CMD:
            return
        else:
            assert False
    
    def handle_wait_cmd(self, event, params):
        if event == EVENT_STOPPED:
            t, image = params
            self.y0 = (t, image)
            return 
        elif event == EVENT_MOVING:
            return  
        elif event == EVENT_CMD:
            t, command = params
            self.u = (t, command)
            return STATE_WAIT_FOR_MOVING
        else:
            assert False
    
    def handle_wait_for_moving(self, event, params): #@UnusedVariable
        if event == EVENT_STOPPED:
            return
        elif event == EVENT_MOVING:
            return STATE_WAIT_FOR_STOP 
        elif event == EVENT_CMD:
            msg = ('Warning: obtained a second command while waiting for the first moving '
                   'image. This should never happen in principle.')
            logger.warning(msg)
            return STATE_WAIT_CMD
        else:
            assert False
            
    def handle_wait_for_stop(self, event, params):
        if event == EVENT_STOPPED:
            # Time to write stuff
            t, image = params
            self.write(Y0=self.y0, U=self.u, Y1=(t, image))
            # update y0 for the next round            
            self.y0 = (t, image)
            return STATE_WAIT_CMD
        elif event == EVENT_MOVING:
            return 
        elif event == EVENT_CMD:
            # Warn this shouldn't happen
            msg = ('Warning: obtained a second command while waiting for the stopped '
                   'image. This should never happen in principle.')
            logger.warning(msg)
            return STATE_WAIT_CMD
        else:
            assert False
    
    def write(self, Y0, U, Y1):
        ty0, y0 = Y0
        tu, u = U
        ty1, y1 = Y1
        
        delay_u_y0 = np.abs(tu - ty0)
        #if tu < ty0:
        #    msg = 'Due to strange raw capture pipeline, the camera was already moving'
        
        # do some checks
        self.queue.append((Y0, U, Y1))
        
    def get_queue(self):
        old_queue = self.queue
        self.queue = []
        return old_queue

class PreProcessor():
    # Define state integers for image capturing states
    def __init__(self, infile, outfile, output_size, nc=10, use_zoom=True,
                 move_time_timeout=0.8, memory=10, min_zoom=100, max_zoom=500):
        self.nc = nc # Number of history images to store in array
        self.last_container = [0] * nc
        self.output_size = output_size
        
        # Decide if zoom should be simulated
        self.use_zoom = use_zoom
        if use_zoom:
            self.zoom_image = zoom_image_center
        else:
            self.zoom_image = no_zoom
        
          
        self.min_zoom = min_zoom
        self.max_zoom = max_zoom
        self.name = infile
        
        self.bag = rosbag.Bag(infile)
        self.out_bag = rosbag.Bag(outfile, 'w')
        
        # Define some variables 
        self.state_capture = state_takeY0
        self.diff_treshold = self.find_dt_threshold()

        self.move_time_timeout = move_time_timeout
        
        self.memory = 10
        
    def finalize(self):
        self.bag.close()
        self.out_bag.close()
        
    def validate_bag(self, outfile): 
        bag = rosbag.Bag(outfile)
        ver_bag = rosbag.Bag('verify.bag', 'w')
        topics = ['Y0', 'U0', 'Y1']
        last_topic = 'Y1'
        seq_error = False
        for topic, msg, t in bag.read_messages(topics=topics):
            if topic == 'Y0':
                if last_topic == 'Y1':
                    pass
                else:
                    logger.info('Wrong sequence')
                    seq_error = True

            if topic == 'U0':
                if last_topic == 'Y0':
                    pass
                else:
                    logger.info('Wrong sequence')
                    seq_error = True
            
            if seq_error:
                ver_bag.write(topic, msg, t)
                        
            if topic == 'Y1':
                seq_error = False

            last_topic = topic
            
        ver_bag.close()
                
    
    def process_bag(self):
        self.i = 0
        from collections import deque
        self.last_images = deque(maxlen=self.memory)
        topics = [topic_image_raw, topic_camera_executed ]
        self.t0 = None
        # Keep track of the actual zoom for the last command and the current zoom    
        current_zoom = self.min_zoom

        state_machine = StateMachine()
        
        tracker = CameraTracker(self.threshold)
        for topic, msg, t in self.bag.read_messages(topics=topics):
            if topic == topic_camera_executed:
                command = msg.data
                state_machine.received_command(t, command)

            if topic == topic_image_raw:
                tracker.push(t, msg)
                if tracker.was_moving():
                    state_machine.received_moving_image(t, msg)
                else:
                    state_machine.received_stopped_image(t, msg)
            
            for data in state_machine.get_queue():
                self.write_stuff(*data)
            
                
                pan, tilt, zoom = command
                
                next_zoom = current_zoom + zoom 
                current_zoom = np.clip(next_zoom, self.min_zoom, self.max_zoom)
                if next_zoom != current_zoom:
                    # We are clipping the zoom, so we ignore it
                    pass # ... 
                
                time_since = t.to_time() - self.t0.to_time() 
                if time_since < self.move_time_timeout:
                    logger.info('ignoring command, timediff is %g ' % time_since)
                else:
                    if (int(zoom) == 0) or self.use_zoom:
                        next_state = self.command_executed(msg, t)
                
                
                next_state = self.cam_image_read(msg, t)
                
            self.state_capture = next_state

    def write_stuff(self, Y0, U, Y1):
        # write to the bag
        pass
        
    def find_dt_threshold(self, ignore_first=100, max_images=300):
        '''
        Finds the value of dy/dt for when the robot is probably moving
        :param infile:    File to read data from
        '''
        logger.info('Calibrating diff_threshold for camera motion...')
        ic = 0

        Y_last = None
        diffs = []
        for _, msg, _ in self.bag.read_messages('/usb_cam/image_raw'):
            if ic > max_images:
                break
            if ic < ignore_first:
                continue
            
            if Y_last is not None:
                diff = images_distance(Y_last, msg)
                diffs.append(diff)
            
            Y_last = msg 
            ic += 1 
            
        diff_threshold = 2 * np.mean(diffs) 
        logger.info('diff_threshold = %g ' % diff_threshold)
        return diff_threshold

    def add_container(self, Y_new):
        """
            Store the last nc images.
            To enable use of a image in the past when writing 
            Y0 since some motions may be to fast for capturing 
            the image after the command is executed.
        """ 
#        for i in range(self.nc - 1):
#            self.last_container[self.nc - i - 1] = self.last_container[self.nc - i - 2]
#        self.last_container[0] = Y_new
        self.last_images.push(Y_new)
        
    def command_executed(self, command, t):
        #pan, tilt, zoom = command
        filled_up = len(self.last_images) == self.memory
        if not filled_up:
            return None
    
        if self.state_capture != PreProcessor.state_takeY0:
#                pdb.set_trace()
            logger.warning('At t = %r , take Y0 outside state' % str(t.to_time))
         #   logger.warning('cmd = %r ' % str(msg.data))
            
      #  logger.debug('Camera Instruction Read: (%s)' % str(msg.data)) 
        #logger.debug('Writing Y0 and U0')
        #logger.debug('Time = %s' % t.to_time())
        Y0 = find_Y0(self.last_container, self.diff_treshold)
        write_img = self.zoom_image(Y0, current_zoom, self.output_size)
        
#            write_img = resize_image(zoom_image(self.last_container[5], self.zoom_last), self.output_size)
        
        self.out_bag.write('Y0', write_img, t)
        self.out_bag.write('U0', msg, t)
        
        # self.t0 is the time when the last command was executed/posted
        self.t0 = t
         
                
        next_state = PreProcessor.state_wait_for_move
        self.move_waits = 0
        
        return next_state
            
    def cam_image_read(self, msg, t):
     
        try:
            diff = compare_images(self.Y_last, msg)
            logger.debug('time: %g       diff = %g' % (t.to_time(), diff))
        except ValueError:
            logger.debug('time: %g       diff = ?' % t.to_time())
                
        
        next_state = self.state_capture            
        if self.state_capture != PreProcessor.state_takeY0:
            if (t.to_time() - self.t0.to_time() > self.move_time_timeout):
                next_state = PreProcessor.state_takeY1
                self.state_capture = next_state
        
        if self.state_capture == PreProcessor.state_wait_for_move:
            diff = compare_images(self.Y_last, msg)
#            logger.debug('Waiting for move state, diff = %g' % diff)
            if diff > self.diff_treshold:
#                logger.debug('Camera detected to be moving')
                next_state = PreProcessor.state_cam_moving
#            self.move_waits += 1

#            if t.to_time() - self.t0.to_time() > self.move_time_timeout:
#                logger.debug('Proceding to state_take_Y1, timeout in state_wait_for_move')
#                next_state = PreProcessor.state_takeY1
                
        if self.state_capture == PreProcessor.state_cam_moving:
            diff = compare_images(self.Y_last, msg)
#            logger.debug('Camera Moving State, diff = %s' % diff)
            if diff < self.diff_treshold:
#                logger.debug('camera has stopped')
                interval = t.to_time() - self.t0.to_time()
                logger.debug('Motion took : %.3fs.' % interval)
                next_state = PreProcessor.state_takeY1
#            self.move_waits += 1

#            if t.to_time() - self.t0.to_time() > self.move_time_timeout:
#                logger.debug('Proceding to state_take_Y1, timeout in state_cam_moving')
#                next_state = PreProcessor.state_takeY1
                
        if self.state_capture == PreProcessor.state_takeY1:
            logger.debug('Writing Y1')
            write_img = self.zoom_image(msg, self.zoom, self.output_size)
            self.out_bag.write('Y1', write_img, self.t0)
            self.i += 1
            
            next_state = PreProcessor.state_takeY0
        
        self.add_container(msg)

        #self.Y_last = msg    # update last read image
        self.zoom_last = self.zoom
        
        return next_state
    
def no_zoom(image, zoom, output_size):
    pim, _, _ = imgmsg_to_pil(image)
    pim = pim.resize(output_size)    
    return pil_to_imgmsg(pim)

def zoom_image_center(image, zoom, output_size):
    """ Zoom an image with zoom center. """
    pim, _, (h, w, _) = imgmsg_to_pil(image)
    z = float(zoom)
    z0 = 100.0 # original size zoom

    x0 = int(w / 2 * (1.0 - z0 / z)) 
    y0 = int(h / 2 * (1.0 - z0 / z)) 
    dx = int(w / 2 * (1.0 + z0 / z)) 
    dy = int(h / 2 * (1.0 + z0 / z)) 
    pim_crop = pim.crop((x0, y0, dx, dy))
    pim_out = pim_crop.resize(output_size)    
    return pil_to_imgmsg(pim_out)

def find_Y0(img_container, diff_threshold):
    '''
    Return the image before the camera started to move
    :param img_container:
    :param diff_threshold:
    '''
    n = len(img_container)
    state = 0
    for i in range(n - 1):
        diff = compare_images(img_container[i], img_container[i + 1])
        if (diff > diff_threshold) and (i < 5):
            # Cam moving, search for stop
            state = 1
        
        if (i >= 5) and (state == 0):
            return img_container[0]
        
        if state == 1:
            # Search for stop in motion (backwards in time)
            if diff < diff_threshold:
                logger.info('stopped at index %g ' % i)
                if compare_images(img_container[i + 1], img_container[i + 2]) < diff_threshold:
                    # camera was stopped as well
                    return img_container[i + 2]
                else:
                    # Camera was stopped now but not on next one, return this image
                    return img_container[i + 1]
    # nothing found, return most recent image
    return img_container[0]

def image_distance(image1, image2):
    """
     Compares two images (/sensor_msgs/Image) and estimates first norm 
     of the difference between the pixel values by calculating the 
     average difference for the set of each <step>'th pixel.
     
     Output
         Average pixel value difference
    """
    im1 = get_image_array(image1).astype('float32')
    im2 = get_image_array(image2).astype('float32')
    return np.mean(np.abs(im1 - im2))
#    normdiff = 0.
#    n = len(image1.data)
#    for i in range(0, n - 1, step):
#        a = int(image1.data[i].encode('hex'), 16) 
#        b = int(image2.data[i].encode('hex'), 16)
#        normdiff += abs(a - b)    
#    return normdiff / n / step 

