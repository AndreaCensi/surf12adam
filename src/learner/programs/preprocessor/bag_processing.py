import rosbag
from . import topic_image_raw, topic_camera_executed
from .camera_tracker import CameraTracker
from .state_machine import StateMachine
 

def read_processed_data(bagfile, zoomer):
    """ yields topic, data, t but data already a numpy array """
    bag = rosbag.Bag(bagfile)
    topics = [topic_image_raw, topic_camera_executed ]
    for topic, msg, t in bag.read_messages(topics=topics):
        if topic == topic_camera_executed:
            command = msg.data
            zoomer.received_command(t, command)

        if topic == topic_image_raw:
            zoomer.received_image(t, msg)
            
        for t, image in zoomer.get_image_queue():
            yield topic_image_raw, image, t

        for t, command in zoomer.get_command_queue():
            yield topic_image_raw, command, t
    bag.close()


def read_Y0UY1_tuples(data_stream, image_distance, threshold):
    """ Reads from data_Stream:
          topic, data, t but data already a numpy array 
          
        yields: (y0,t), (u,t), (y1, t)
    """
    tracker = CameraTracker(threshold, image_distance)
    state_machine = StateMachine() 
    for topic, msg, t in data_stream:
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
            yield data
    
