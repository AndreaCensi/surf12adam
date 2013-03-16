import numpy as np
import numpy.random
import Queue
from compmake import comp
from compmake.ui.console import compmake_console
import pdb

class LearnerHub():
    def __init__(self, itterator):
        self.clients = []
        self.gloal_itterator = itterator
    
    def register_estimator(self, constraint):
        Q = Queue.Queue()
        client = {'queue':Q, 'function':constraint}
        self.clients.append(client)
        index = self.clients.index(client)
        return index
        
    def read_filtered(self, index):
        print('reading filtered %d' % index)
        self.fill_queue(index)
        yield self.clients[index]['queue'].get()
        
#        shape = np.array(command).shape
#        if len(shape) == 2 and shape[0] == 2:
#            command_lower = command[0]
#            command_upper = command[1]
#        else:
#            command_lower = command
#            command_upper = command
    
    def fill_queue(self, index):
        print('filling up queue %d' % index)
        while not self.clients[index]['queue'].empty():
            item = self.gloal_itterator.next()
            _, command, _ = item
            handled = False
            for client in self.clients:
                if client['function'](command):
                    handled = True
                    client['queue'].put(item)
                    
            if not handled:
                i = self.register_estimator(command)
                self.clients[i]['queue'].put(item)

    def handle_unknown(self, item):
        _, command, _ = item 
        self.register_estimator(command)

def read_all(n):
    for _ in range(n):
        yield ('Y0', np.random.randint(0, 4), 'Y1')

def demo_topic(name, itterator):
    for items in itterator:
        _, command, _ = items
        print('demo_topic_' + name + ' read item: ' + str(command))
        
if __name__ == '__main__':
    
    hub = LearnerHub(read_all(100))
    def constr0(command, state=None):
        if command == 0:
            return True
        else:
            return False
        
    def constr1(command, state=None):
        if command == 1:
            return True
        else:
            return False
        
    def constr2(command, state=None):
        if command == 2:
            return True
        else:
            return False
        
    def constr3(command, state=None):
        if command == 3:
            return True
        else:
            return False
        
    hub.register_estimator(constr0)
    hub.register_estimator(constr1)
    hub.register_estimator(constr2)
    hub.register_estimator(constr3)
    
    pdb.set_trace()
    
    comp(demo_topic, '0', hub.read_filtered(0))
    comp(demo_topic, '1', hub.read_filtered(1))
    comp(demo_topic, '2', hub.read_filtered(2))
    comp(demo_topic, '3', hub.read_filtered(3))
    compmake_console(0)
