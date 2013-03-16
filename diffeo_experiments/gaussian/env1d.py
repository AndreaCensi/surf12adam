
import numpy as np
import numpy.random #@UnusedImport
#import pylab
import pdb
import sys
import scipy
from scipy.optimize.minpack import leastsq
from reprep import Report
import pylab

class env1d():
    def __init__(self):
        report = Report('id', caption='env1d')
        self.N = 1000
        self.res = 10
        
        x = np.linspace(0, 10, self.N * self.res)
        self.E = scipy.convolve(np.random.ranf(len(x)),
                                np.ones(self.res * 20) / self.res * 20,
                                mode='same')
        plot_env(report, x, self.E)
        
        self.commands = [-2.5, 2.0]
        self.n_sampels = [0, 0]
        self.sensels = [30, 31]
        self.state = self.N / 2
        
        self.plot_y = False
        self.plot_e = False
        
        self.size = 60
        self.area = 9
        self.s = range(self.size)
        
        self.clean()
        lsize = 20
        sensor_noise = 0
        actuator_noise = 0
        self.run_learning(lsize, actuator_noise=actuator_noise, sensor_noise=sensor_noise)
        report.text('info0', ('Learning size: \t\t%g \nActuator noise: \t%g ' + 
                             '\nSensor noise: \t\t%g') % (lsize, actuator_noise, sensor_noise))
        
        report.text('commands', str(self.commands))
        self.summarize(report, 0)
        
        
        self.state = self.N / 2
        self.clean()
        lsize = 100
        sensor_noise = 0
        actuator_noise = 2
        self.run_learning(lsize, actuator_noise=actuator_noise, sensor_noise=sensor_noise)
        report.text('info1', ('Learning size: \t\t%g \nActuator noise: \t%g ' + 
                             '\nSensor noise: \t\t%g') % (lsize, actuator_noise, sensor_noise))
        self.summarize(report, 1)
        
        
        self.state = self.N / 2
        self.clean()
#        lsize = 1000
        sensor_noise = 2
        actuator_noise = 0
        self.run_learning(lsize, actuator_noise=actuator_noise, sensor_noise=sensor_noise)
        report.text('info2', ('Learning size: \t\t%g \nActuator noise: \t%g ' + 
                             '\nSensor noise: \t\t%g') % (lsize, actuator_noise, sensor_noise))
        self.summarize(report, 2)
        
        report.to_html('env1d.html')
#        pylab.show()
        
    def clean(self):
        self.e1_lists = [[[], []]] * len(self.sensels)
        self.n_sampels = [0, 0, 0]
        
    def run_learning(self, n, **kwargs):
        t = 0
        step = 50
        sys.stdout.write('|')
        for _ in range(n):
            t += 1
            if t > n / step:
                t = 0
                sys.stdout.write('=')
            command = np.random.randint(0, len(self.commands))
            y0, u0, y1 = self.get_images(self.commands[command], **kwargs)
            self.update(y0, u0, y1)
        sys.stdout.write('|')
            
    def get_sub_image(self, lower, upper=None):
        pixel = int(lower * self.res)
        if upper is None:
            upper = lower + self.size
            
        pixel_upper = int(upper * self.res)
        ran = range(pixel, pixel_upper, self.res)
        return self.E[ran]
        
        
    def get_images(self, command, sensor_noise=0.0, actuator_noise=0):
        state = self.state
#        E = self.E
#        size = self.size
#        y0 = E[state:state + size]
        y0 = self.get_sub_image(state) + (np.random.ranf() - .5) * sensor_noise
        try:
            new_state = state + command + (np.random.ranf() - .5) * actuator_noise
        except:
            pdb.set_trace()  
#        y1 = E[new_state:new_state + size]
        y1 = self.get_sub_image(new_state) + (np.random.ranf() - .5) * sensor_noise
        self.state = new_state
        return y0, command, y1

#    def init_y_fig(self):
#        pylab.figure(1)
#        pylab.subplot(2, 2, 1)
#        pylab.hold(True)
#        pylab.title('Image y0')
#        pylab.ylabel('Command 0')
#        
#        
#        pylab.subplot(2, 2, 2)
#        pylab.hold(True)
#        pylab.title('Image y1')
#        
#        
#        pylab.subplot(2, 2, 3)
#        pylab.hold(True)
#        pylab.title('Image y0')
#        pylab.ylabel('Command 1')
#        
#        
#        pylab.subplot(2, 2, 4)
#        pylab.hold(True)
#        pylab.title('Image y1')

    def update(self, y0, command, y1):
        if self.plot_y:
            pass
#            pylab.figure(1)
#            ## Handle y0
#            #   Find plot 
#            if command == self.commands[0]:
#                pylab.subplot(2, 2, 1)
#            elif command == self.commands[1]:
#                pylab.subplot(2, 2, 3)
#                
#            pylab.plot(self.s, y0)
#            
#            ## Handle y1
#            #   Find plot
#            if command == self.commands[0]:
#                pylab.subplot(2, 2, 2)
#                
#            elif command == self.commands[1]:
#                pylab.subplot(2, 2, 4)
#        
#            pylab.plot(self.s, y1)
            
        
        use = self.sensels
        
        # Determine collumn for error plot
        if command == self.commands[0]:
            self.n_sampels[0] += 1
            col = 1
        elif command == self.commands[1]:
            self.n_sampels[1] += 1
            col = 2
        
        for p in self.s:
            if p in use:
                e1 = y0[p - self.area / 2:p + self.area / 2 + 1] - y1[p]
                try:
#                    self.e1_lists[use.index(p)][col - 1].append(np.abs(e1)) TODO:
                    self.e1_lists[use.index(p)][col - 1].append(e1)
                except:
                    pdb.set_trace()
                if self.plot_e:
                    pass
#                    pylab.figure(2)
#                    pylab.subplot(len(self.sensels), len(self.commands), use.index(p) * 2 + col)
#                    pylab.plot(range(self.area), np.abs(e1),
#                               color='red',
#                               linestyle='dashed')
                
    def summarize(self, report, index):
        
        func1 = lambda v, x, mu: v[0] + v[1] * np.exp(-np.abs((np.array(x) - mu) / v[2]))
        full1 = lambda v, x: v[0] + v[1] * np.exp(-np.abs((np.array(x) - v[2]) / v[3]))

        func2 = lambda v, x, mu: v[0] + v[1] * np.exp(-((x - mu) / v[2]) ** 2) + v[3] * np.exp(-np.abs((np.array(x) - mu) / v[4]))
        full2 = lambda v, x: v[0] + v[1] * np.exp(-((x - v[2]) / v[3]) ** 2) + v[4] * np.exp(-np.abs((np.array(x) - v[2]) / v[5]))

        func3 = lambda v, x, mu: v[0] + v[1] * np.exp(-((np.array(x) - mu) / v[2]) ** 2)
        full3 = lambda v, x: v[0] + v[1] * np.exp(-np.abs((np.array(x) - v[2]) / v[3]) ** 2)
        
        # Force E to go to zero at minimum
        func4 = lambda v, x, mu: v[0] - v[0] * np.exp(-np.abs((np.array(x) - mu) / v[1]))
        full4 = lambda v, x: v[0] - v[0] * np.exp(-np.abs((np.array(x) - v[1]) / v[2]))
        
        funce = lambda v, x, y, mu: (func1(v, x, mu) - y)
        fulle = lambda v, x, y: (full1(v, x) - y)
        use = self.sensels
#        pylab.figure()
        for p in self.s:
            if p in use:
                f = report.figure(cols=2)
                e1_list = self.e1_lists[use.index(p)]
                for c, clist in enumerate(e1_list):
                    E1 = np.sum(np.array(clist), axis=0) / self.n_sampels[c]
                    V = np.std(np.array(clist), axis=0)
                    
                    v0 = np.array([1, -1, self.area / 2, 1])
                    mu = np.argmin(V)
                    v, _ = leastsq(funce, v0, args=(range(self.area), V, mu), maxfev=10000)
                    
                    v0 = [v[0], v[1], mu , v[3]]
                    v, _ = leastsq(fulle, v0, args=(range(self.area), V), maxfev=10000)
                    
                    x = np.linspace(0, self.area - 1, 100)
                    d = x[np.argmin(full1(v, x))] - self.area / 2
                    
#                    pylab.subplot(len(self.sensels), len(self.commands), use.index(p) * 2 + c + 1)
                    with f.plot('p%g_c%g' % (p, c), caption='Pixel: %g  Command: %g  Samples: %g' % (p, c, self.n_sampels[c])) as pylab:
                        pylab.hold(True)
                        pylab.errorbar(range(self.area), E1 * 0, V)
                        pylab.xlim((0, self.area))
                        
                        pylab.plot(x, full1(v, x), color='g')
                        
                    report.text('test%g_cmd_p%g_c%g' % (index, p, c), str(d))
#                        pylab.title('c1 = %g, c2 = %g' % (v[1], v[3]))
                    
                
                
    def init_errorpl(self):
#        pylab.figure(2)
#        pylab.subplot(len(self.sensels), len(self.commands), 1)
        pass
        
def plot_env(report, x, E):
    f = report.figure()
    with f.plot('environment', caption='Plot of environment') as pylab:
        pylab.plot(x, E)
        pylab.title('Environment')




    

env1d()


