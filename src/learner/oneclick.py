'''
Created on Sep 24, 2012

@author: adam
'''
import subprocess
import time
import atexit
from subprocess import PIPE
from optparse import OptionParser
import pdb
import os
import yaml

class OneClick():
    def __init__(self, name, outdir, command_list, duration, size, area):
        self.outdir = outdir
        self.name = name
        self.command_list = command_list
        self.duration = duration
        self.size = size
        
#        print('$OUT = ' + os.environ["OUT"])
        
        if not os.path.exists(outdir):
            os.mkdir(outdir)
        
        os.chdir(self.outdir)
        if not os.path.exists('raw-capture'):
            os.mkdir('raw-capture')
        if not os.path.exists('processed_data'):
            os.mkdir('processed_data')
        
        
    
    def run_capture_data(self):
        self.start_ros()
        self.start_usb_cam()
        self.start_image_viewer()
        self.start_camera_actuator()
        outname = self.outdir + 'raw-capture/' + self.name + '.raw.bag'
        self.start_record(outname)
        self.start_command_generator(command_list=self.command_list, duration=self.duration)
        t0 = time.time()
        while time.time() - t0 < self.duration:
            time.sleep(1)
            pass
        info('Shutting Down data capture, time duration expired.')
        self.terminate_data_capture()
        
    def terminate_data_capture(self):
        info('Terminating processes')
        for name in [self.roscore, self.usb_cam_node, self.image_viewer, self.camera_actuator, self.generator, self.record]:
            try:
#                name.terminate()
                name.send_signal(subprocess.signal.SIGINT)
                info('successfully shut down ' + str(name))
            except:
                warning(str(name) + ' already stopped or not initialized from OneClick')

    def start_ros(self):
        self.roscore = subprocess.Popen(["roscore"])
        time.sleep(5)
        if self.roscore.poll() is None:
            info('roscore started successfully by OneClick')
    
    def start_usb_cam(self):
        subprocess.call(["rosparam", "set", "/usb_cam/pixel_format", "yuyv"])
        self.usb_cam_node = subprocess.Popen(["rosrun", "usb_cam", "usb_cam_node"], stderr=PIPE)
        time.sleep(10)
        
    def start_image_viewer(self):
        self.image_viewer = subprocess.Popen(["rosrun", "image_view", "image_view", "image:=/usb_cam/image_raw"])
    
    def start_camera_actuator(self):
        self.camera_actuator = subprocess.Popen(["rosrun", "camera_actuator", "camera_actuator_node"])
        time.sleep(6)
        
    def start_record(self, outname):
        info('Starting to record data')
        self.record = subprocess.Popen(["rosbag", "record", "-O", outname, "/usb_cam/image_raw", "/logitech_cam/camera_executed"])
        
    def start_command_generator(self, command_list, duration):
        self.generator = subprocess.Popen(["rosrun", "camera_actuator", "command_generator.py", "-c", str(command_list), "-t", str(duration)])
        time.sleep(12)

    def run_preprocess(self):
        inname = self.outdir + 'raw-capture/' + self.name + '.raw.bag'
        size_str = str(self.size[0]) + 'x' + str(self.size[1])
        outname = self.outdir + 'processed_data/' # + '.processed.bag' #+ self.name + size_str 
        self.preprocessor = subprocess.Popen(["preprocess", "-i", inname, "-o", outname])
        self.preprocessor.wait()
        stream = file(self.outdir + 'processed_data/' + 'oneclick.streams.yaml', 'w')
        yaml.dump([{'code': ['diffeoplan.library.logs.BagStream',
           {'files': [outname + self.name + '-' + size_str + '.processed.bag']}],
          'id': self.name,
          'desc': 'Streams for oneclick, pt64, 160x120 resolution.'}], stream)
        
    def run_learning(self):
        stream = "oneclick_diffeo"
        learner = "n35s"
        self.learner = subprocess.Popen(["dp", "plearn", "-s", stream, "-l", learner, "-c", "clean *summarize*; parmake "])

def info(string):
    print('\033[92mINFO:OneClick:     ' + string + '\033[0m')

def warning(string):
    print('\033[93mWARNING:OneClick:  ' + string + '\033[0m')
    

#if __name__ == '__main__':
def one_click_main():
    usage = "usage: %oneclick -O output -c command_list -t duration"
    parser = OptionParser(usage=usage, version="%prog 1.0")
    parser.add_option("-n", "--name", default='oneclick_diffeo',
                      help="name")
    parser.add_option("-O", "--output", default=os.environ["HOME"] + '/out/',
                      help="output folder") 
    parser.add_option("-c", "--command_list", default=[[64, 0, 0], [-64, 0, 0], [0, 64, 0], [0, -64, 0]],
                      help="List of available commands")
    parser.add_option("-t", "--duration", default=10,
                      help="Time duration for recording data")
    parser.add_option("-s", "--size", default='[160, 120]',
                      help="Size of processed image")
    parser.add_option("-a", "--area", default='[8, 8]',
                      help="Size of search area")
    
    options, _ = parser.parse_args()
    
    size = eval(options.size)
    area = eval(options.area)
    
    program = OneClick(name=options.name,
                       outdir=options.output,
                       command_list=options.command_list,
                       duration=int(options.duration),
                       size=size,
                       area=area)
    atexit.register(program.terminate_data_capture)
    program.run_capture_data()
    program.run_preprocess()
    program.run_learning()
    info('Done')
    
    

