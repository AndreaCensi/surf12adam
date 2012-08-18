from . import logger
class CameraTracker:
    
    def __init__(self, threshold, distance):
        self.last_stopped = None
        self.last = None
        self.threshold = threshold
        self.distance = distance
    
    def push(self, t, image):
        if self.last is None:
            # this is the first image
            # we consider it moving
            pass
        else:
            # we have some image to compare
            diff = self.distance(image, self.last)
            logger.debug(diff)
            stopped = diff < self.threshold 
            if stopped:
                # this is a stopped image, so we remember it
                self.last_stopped = image
                self.last_stopped_t = t
            self.last_stopped = stopped
        self.last = image
        self.last_t = t
        
    def was_stopped(self):
        """ Return true if the last image was moving. """
        assert self.last is not None
        return self.last_stopped
    
    def has_last_stopped(self):
        return self.last_stopped is not None
    
    def get_last_stopped(self):
        assert self.has_last_stopped()
        return self.last_stopped
