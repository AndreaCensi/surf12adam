from .. import logger
import time

class LogRecord(object):
    
    def __init__(self, name, timestamp, string):
        self.name = name
        self.timestamp = timestamp
        self.string = string
    
    def __str__(self):
        return '%s: %s' % (self.name, self.string)
        
    
class WithInternalLog(object):
    
    def __init__(self, name=None):
        self.log_lines = [] # log records
        self.set_log_output(True)
        self.children = {}
        if name is None:
            name = self.__class__.__name__ # don't call str() yet
        self.name = name
        
    def set_name_for_log(self, name):
        self.name = name
        
    def log_add_child(self, id_child, child):
        assert isinstance(child, WithInternalLog)
        self.children[id_child] = child
        child.set_name_for_log(id_child)
    
    def set_log_output(self, enable):
        """ Enable or disable instantaneous on-screen logging.
            If disabled, things are still memorized. """
        self.log_output_enabled = enable
    
    def info(self, s):
        """ Logs a string; saves it for visualization. """
        record = LogRecord(name=self.name, timestamp=time.time(), string=s)
        self.log_lines.append(record)
        if self.log_output_enabled:
            logger.info(record.__str__())
    
    def get_log_lines(self):
        """ Returns a list of LogRecords """
        lines = list(self.log_lines)
        for child in self.children.values():
            lines.extend(child.get_log_lines())
        lines.sort(key=lambda x: x.timestamp)
        return lines
    
    def get_raw_log_lines(self):
        """ Returns a list of strings """
        raw = map(LogRecord.__str__, self.get_log_lines())
        return raw
