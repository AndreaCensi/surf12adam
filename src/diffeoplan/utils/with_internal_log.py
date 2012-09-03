from .. import logger
import time

class WithInternalLog(object):
    def __init__(self):
        self.log_lines = [] # log lines

    def info(self, s):
        """ Logs a string; saves it for visualization. """
        self.log_lines.append((time.time(), s))
        logger.info("%s: %s" % (self, s))
        

