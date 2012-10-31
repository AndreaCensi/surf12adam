#!/usr/bin/env python
import rospy
import time
import sensor_msgs.msg
import std_msgs.msg
from std_msgs.msg import String
import camera_actuator.srv
import sys
import pdb
from diffeoplan.library.logs.rosbag.bag_reader import get_image_array
from PIL import Image, ImageTk
import Tkinter as TK
from Tkinter import Label
#import UI
import threading
from threading import Thread
import numpy as np
import pylab
import pickle

def main():
#    print('starting')
    dpClient = DiffeoPlanner()

commands = {'left': 3,
            'right': 1,
            'up': 0,
            'down': 2}
state_map = {0: (0, 1), 1: (1, 0), 2:(0, -1), 3:(-1, 0)}
area = {'left': 12,
            'right': 12,
            'up': 5,
            'down': 5}

class DiffeoPlanner():
    def __init__(self):
        self.node_name = 'diffeoplan_client'
        self.server_name = 'diffeoplan_server'
        rospy.init_node(self.node_name)
        
        # Subscribe to /usb_cam/image_raw
        rospy.Subscriber('/usb_cam/image_raw', sensor_msgs.msg.Image, self.incoming_image)
        
        # Initiate publisher for set goal and current
        self.goal_pub = rospy.Publisher(self.server_name + '/set_goal', sensor_msgs.msg.Image)
        self.current_pub = rospy.Publisher(self.server_name + '/set_current', sensor_msgs.msg.Image)
        
        # Create service to set goal
        goal_service = rospy.Service(self.node_name + '/set_goal',
                                     camera_actuator.srv.voidService,
                                     self.set_goal)
        
        # Initiate service to ask for plan
        self.get_plan = rospy.ServiceProxy(self.server_name + '/get_plan',
                                           camera_actuator.srv.planService)
        
        # Initiate service to execute plan
        self.executePlan = rospy.ServiceProxy('/logitech_cam/executePlan',
                                           camera_actuator.srv.planCommand)
        
        # Initiate service to get distance array
        self.get_p_d = rospy.ServiceProxy(self.server_name + '/get_predicted_distance',
                                           camera_actuator.srv.floatArrayService)
        
        # Service to get current distance
        self.get_distance = rospy.ServiceProxy(self.server_name + '/get_distance',
                                           camera_actuator.srv.floatService)

        def line_match(strA, strB):
            if strA[:len(strB)] == strB:
                return True
            else:
                return False
            
        pylab.figure()
        
        print('Node started')
        
        while not rospy.is_shutdown():
            line = sys.stdin.readline().strip('\n')
            handled = False
            successful = False
#            pdb.set_trace()
            if line_match(line, 'run'):
                # Register handling of command
                handled = True
                
                run_result = self.run_demo(line)
                pickle.dump(run_result, open('last_result.pickle', 'wb'))
                
                
                
                # Register command handling successful
                successful = True
            
            
            
            if line_match(line, 'dshow'):
                # Register handling of command
                handled = True
                
                run_result = pickle.load(open('last_result.pickle'))
                self.plot_distance(run_result)
                
                # Register command handling successful
                successful = True
            
            if line_match(line, 'goal'):
                # Register handling of command
                handled = True
                
                self.set_goal()
                
                # Register command handling successful
                successful = True
                
            if line_match(line, 'start'):
                # Register handling of command
                handled = True
                
                self.set_current()
                
                # Register command handling successful
                successful = True
                
            if line_match(line, 'show'):
                # Register handling of command
                handled = True
                
                self.show_images()
                
                # Register command handling successful
                successful = True
                
            if line_match(line, 'demo'):
                handled = True
                
                self.track_state_home()
                
                self.distance_grid = self.scan_environment_distances()
                pickle.dump(self.distance_grid, open('distance_grid.pickle', 'wb'))
                
                all_results = self.demo_with_map()
                pickle.dump(all_results, open('all_results.pickle', 'wb'))
                
#                self.distance_grid = pickle.load(open('distance_grid.pickle'))
#                all_results = pickle.load(open('all_results.pickle'))
                
                
                self.draw_map()
                self.draw_threshold()
                self.draw_results(all_results)
                pylab.savefig('final.png')
                
                # Register command handling successful
                successful = True
                
            if line_match(line, 'scan'):
                # Register handling of command
                handled = True
                
                self.distance_grid = self.scan_environment_distances()
                self.draw_map()
                self.draw_threshold()
                pylab.savefig('test.png')
                
                # Register command handling successful
                successful = True
                
            if line_match(line, 'call'):
                # Register handling of command
                handled = True
                
                args = line.split(' ')
                if len(args) > 2:
                    call = rospy.ServiceProxy(self.server_name + '/' + args[1],
                                               camera_actuator.srv.imageService)
                    try:
                        call(args[2])
                        # Register command handling successful
                        successful = True
                    except:
                        print('Failed to call service')
                else:
                    print('Not enough arguments')
                    
            if line_match(line, 'set'):
                # Register handling of command
                handled = True
                
                args = line.split(' ')
                if len(args) > 2:
                    try:
                        rospy.set_param(self.server_name + '/' + args[1], eval(args[2]))
                        # Register command handling successful
                        successful = True
                    except:
                        pass
                else:
                    print('Not enough arguments')
            
            # Take care of unknown or failed commands
            if not handled:
                print('Unknown command: ' + line)
            if not successful:
                print('Failed to execute command')
        
    def run_demo(self, line):
        args = line.split(' ')
        if len(args) <= 1:
            n = 10
        else:
            n = args[1]
        
        itter_results = []
        for i in range(n):
            print('run_one ' + str(i))
#            pdb.set_trace()
            y_0 = get_image_array(self.y_current)
            
            plan, plan_executed = self.run_one()
            if plan == (-1,):
                break
            
            dist0 = self.get_distance(0).res
            pred_dist = self.get_p_d(plan).array
            
            self.set_current()
            
            dist = self.get_distance(0).res
            
            
            itter_results.append({'i':i,
                                  'plan_found':tuple(plan),
                                 'plan_executed':plan_executed,
                                 'start_distance':dist0,
                                 'final_distance':dist,
                                 'predicted_distances':pred_dist,
                                 'y_0':y_0,
                                 'y_1':get_image_array(self.y_current)})
        run_result = {'itter_results':itter_results}
        return run_result
        
    def plot_distance(self, run_result):
        Y_vals = np.zeros(len(run_result['itter_results']) + 1)
        
        for itter_result in run_result['itter_results']:
            i = itter_result['i']
            pred_dist = itter_result['predicted_distances']
            print(str(pred_dist))
            dist0 = itter_result['start_distance']
            dist = itter_result['final_distance']
            Y_vals[i] = dist0
            if i == len(Y_vals) - 1:
                Y_vals[i + 1] = dist
            x_vals = range(i, i + len(pred_dist) + 1)
            y_vals = (dist0,) + pred_dist
            l_pred_style = {'color':'gray'}
            pylab.plot(x_vals, y_vals, **l_pred_style)
            
            l_style = {'color':'b',
                       'linewidth':3}
            pylab.plot(range(i + 1), Y_vals[:i + 1], **l_style)
            pylab.savefig('dist_it' + str(i) + '.png')
            
        
    def show_images(self):
        # Initiate services to get images
        get_goal_service = rospy.ServiceProxy(self.server_name + '/get_goal',
                                           camera_actuator.srv.imageService)
        get_current_service = rospy.ServiceProxy(self.server_name + '/get_current',
                                           camera_actuator.srv.imageService)
        im_goal = get_goal_service(0)
        im_current = get_current_service(0)
        
        np_goal = get_image_array(im_goal.image)
        pil_goal = Image.fromarray(np_goal)
        
        np_current = get_image_array(im_current.image)
        pil_current = Image.fromarray(np_current)

        self.ui = UI(pil_goal, pil_current) 

    def run_one(self):
        try:
            # Update current image
            self.set_current()
            
            res = self.get_plan([])
            print(str(res.plan))
            executed = ()
            # Execute the first step in the plan
            if len(res.plan) > 0:
                self.executePlan([res.plan[0]])
                executed = (res.plan[0],)
            
            return res.plan, executed
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def incoming_image(self, msg):
        self.last_image = msg
    
    def set_goal(self):
#        print()
        self.y_goal = self.last_image
        self.goal_pub.publish(self.y_goal)
        
    def set_current(self):
#        print()
        self.y_current = self.last_image
        self.current_pub.publish(self.y_current)
    
    def scan_environment_distances(self):
        start_cmd1 = [commands['left']] * area['left']
        start_cmd2 = [commands['down']] * area['down']
        
        self.set_goal()
        
        self.executePlan(start_cmd1)
        self.executePlan(start_cmd2)
        
        nx = area['left'] + area['right']
        ny = area['up'] + area['down']
        get_distance = self.get_distance
        
        
        # Initiate distance grid
        distance_grid = np.zeros((nx + 1, ny + 1))
        
        # get the very first distance
        self.set_current()
        dist = get_distance(0.0)
#        pdb.set_trace()
        distance_grid[0, 0] = dist.res
        
        for j in range(0, ny + 1):
            if j % 2 == 1:
                for i in range(0, nx):
                    # get distance
                    self.set_current()
                    dist = get_distance(0.0)
                    distance_grid[i, j] = dist.res
                    
                    # Move to next position
                    self.executePlan([commands['left']])
                    
                # get last distance on row
                dist = get_distance(0.0)
                distance_grid[nx, j] = dist.res
                    
            else:
                for i in range(nx, 0, -1):
                    # get distance
                    self.set_current()
                    dist = get_distance(0.0)
                    distance_grid[i, j] = dist.res
                    
                    # Move to next position
                    self.executePlan([commands['right']])
                    
                # get first distance on row
                dist = get_distance(0.0)
                distance_grid[0, j] = dist.res
                
            # Move to next position
            self.executePlan([commands['up']])
            
        
        
#        pickle.dump(distance_grid, open('distance_grid.pickle'))
#        pdb.set_trace()
        return distance_grid

    def draw_map(self):
        distance_grid = self.distance_grid
        levels = np.linspace(0, np.max(distance_grid))
        X, Y = np.meshgrid(range(-area['left'], area['right'] + 1), range(-area['down'], area['up'] + 1))
#        pylab.imshow(distance_grid.transpose())
        pylab.contourf(X.transpose(),
                       Y.transpose(),
                       distance_grid,
                       levels=levels)
        pylab.colorbar()
        
    def draw_threshold(self):
        distance_grid = self.distance_grid
        try:
            thresh = rospy.get_param(self.server_name + '/planning_thresholds')
            X, Y = np.meshgrid(range(-area['left'], area['right'] + 1), range(-area['down'], area['up'] + 1))
            min_lev = [0, thresh[0], 1]
            pylab.contour(X.transpose(), Y.transpose(), distance_grid, levels=min_lev, colors='k')    
        except:
            print('failed to draw thresholds')
        
    def init_map(self):
        pass
        
    def track_state_move(self, plan):
        state0 = self.state
        new_state = state0
        for p in plan:
            new_state = new_state + state_map[p]
            self.executePlan([p])
            self.state = new_state
            
    def track_state(self, plan):
        state0 = self.state
        new_state = state0
        for p in plan:
            new_state = new_state + state_map[p]
#            self.executePlan([p])
            self.state = new_state
    
    def track_state_home(self):
#        try:
        print('calling for home')
        call = rospy.ServiceProxy('/logitech_cam/home', camera_actuator.srv.voidService)
        call(0)
        self.state = np.array([0, 0])
        print('homing successful')
        return True
#        except:
#            print('homing failed')
#            return False
    
    
    def demo_with_map(self):
        get_distance = rospy.ServiceProxy(self.server_name + '/get_distance',
                                           camera_actuator.srv.floatService)
#        pdb.set_trace()
        
        all_results = []
        
        starts = [(1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2),
                  (1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0),
                  (1, 1, 2),
                  (0, 3, 3, 3),
                  (2, 2, 3, 3, 3, 3)]
        for start in starts:
            self.track_state_home()
            itter_results = []
            self.track_state_move(start)
            executed = []
            for i in range(20):
                state0 = self.state 
                dist0 = get_distance(0)
                print('run_one ' + str(i))
                plan, plan_executed = self.run_one()
                if plan == (-1,):
                    break
                itter_results.append({'plan_found':tuple(plan),
                                     'plan_executed':plan_executed,
                                     'start_state':state0,
                                     'start_distance':dist0,
                                     'final_distance':get_distance(0)})
                
                time.sleep(1)
                self.track_state(plan_executed)
                for p in plan_executed:
                    executed.append(p)
            run_result = {'start_move': start,
                          'itter_results':itter_results,
                          'plan':executed}
            all_results.append(run_result)
        return all_results
    def draw_results(self, all_results):
        ## Plot the map
        # Define some styles for plots
        found_style = {'linewidth':1.5,
                       'marker':'o',
                       'markersize':4,
                       'color':'w'}
        executed_style = {'linewidth':4,
                          'marker':'.',
                          'markersize':15,
                          'color':'b'}
#        pdb.set_trace()
        for r, run_result in enumerate(all_results):
            
            for i, itter_result in enumerate(run_result['itter_results']):
                
                # Plot the executed part
                self.plot_plan_line(itter_result['start_state'],
                               itter_result['plan_executed'],
                               **executed_style)
                
                # Plot the found plan
                self.plot_plan_line(itter_result['start_state'],
                               itter_result['plan_found'],
                               **found_style)
                
                pylab.savefig('map_' + str(r) + '_it' + str(i) + '.png')
    
    def plot_plan_line(self, state0, plan, **lineargs):
        new_state = state0
        states = [state0]
        for p in plan:
            new_state = new_state + state_map[p]
            states.append(new_state)
#            self.executePlan([p])
            self.state = new_state
        ret = np.zeros((2, len(states)))
#        pdb.set_trace()
        for i, st in enumerate(states):
            ret[:, i] = st
        pylab.plot(ret[0, :], ret[1, :], **lineargs)

class UI(Thread):
    def __init__(self, img, imc):
        threading.Thread.__init__(self)
        self.img = img
        self.imc = imc
#        pdb.set_trace()
        self.start()
        
    def run(self):
        self.ui = TK.Tk()
        tk_goal = ImageTk.PhotoImage(self.img)
        self.panel_goal = Label(self.ui, border=0, image=tk_goal)
        self.panel_goal.pack()
        
        tk_current = ImageTk.PhotoImage(self.imc)
        self.panel_current = Label(self.ui, border=0, image=tk_current)
        self.panel_current.pack()
#        pass
        self.ui.mainloop()
            

    
if __name__ == '__main__':
    main()
