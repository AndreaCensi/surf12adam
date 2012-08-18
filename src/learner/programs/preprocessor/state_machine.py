from . import logger
import numpy as np
 
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
        self.state = next_state
        
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
#            logger.debug('handle_wait_for_moving -> STATE_WAIT_FOR_STOP')
            return STATE_WAIT_FOR_STOP 
        elif event == EVENT_CMD:
            msg = ('Warning: obtained a second command while waiting for the first moving '
                   'image. This should never happen in principle.')
            logger.warning(msg)
            return STATE_WAIT_CMD
        else:
            assert False
            
    def handle_wait_for_stop(self, event, params):
#        logger.debug('handle_wait_for_stop')
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
                   'image. This should never happen in principle. Last command was: '
                   '%s and this command was: %s' % (self.u, params))
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
#        logger.debug('append to queue')
        self.queue.append((Y0, U, Y1))
        
    def get_queue(self):
        old_queue = self.queue
        self.queue = []
        return old_queue
