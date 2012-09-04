from . import topic_image_raw, topic_camera_executed, logger
from .camera_tracker import CameraTracker
from .state_machine import StateMachine
import rosbag
 

def read_processed_data(bagfile, zoomer):
    """ yields topic, data, t but data already a numpy array """
    bag = rosbag.Bag(bagfile)
    i = 0
    topics = [topic_image_raw, topic_camera_executed]
    for topic, msg, t in bag.read_messages(topics=topics):
        i += 1
        if topic == topic_camera_executed:
            zoomer.received_command(t, msg)

        if topic == topic_image_raw:
            zoomer.received_image(t, msg)
            
        for _, image in zoomer.get_image_queue():
            yield topic_image_raw, image, t

        for _, command_msg in zoomer.get_command_queue():
            yield topic_camera_executed, command_msg, t
    
    bag.close()


def read_Y0UY1_tuples(data_stream, image_distance, threshold):
    """ Reads from data_Stream:
          topic, data, t but data already a numpy array 
          
        yields: (y0,t), (u,t), (y1, t)
    """
    tracker = CameraTracker(threshold, image_distance)
    state_machine = StateMachine() 
    i = 0
    for topic, msg, t in data_stream:
        if topic == topic_camera_executed:
            state_machine.received_command(t, msg)
        
        if topic == topic_image_raw:
            tracker.push(t, msg)
            if tracker.was_stopped():
                state_machine.received_stopped_image(t, msg)
            else:
                state_machine.received_moving_image(t, msg)
        
        for data in state_machine.get_queue():
            t0 = data[0][0].to_time()
            logger.info('Yeild tuple # %g :(%f, Y0), (+%f, %s), (+%f, Y1)' % 
                        (i, t0,
                         data[1][0].to_time() - t0,
                         data[1][1],
                         data[2][0].to_time() - t0))
            i += 1
            yield data
    logger.info('read_Y0UY1_tuples EOF')
        
    
