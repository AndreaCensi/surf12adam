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
from diffeoplan.library.logs.rosbag.ros_conversions import *

def main():
#    print('starting')
    dpClient = DiffeoPlanner()

commands = {'left': 3,
            'right': 1,
            'up': 0,
            'down': 2}
state_map = {0: (0, 1), 1: (1, 0), 2:(0, -1), 3:(-1, 0)}
area = {'left': 6,
            'right': 6,
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
        
        # Publisher for distance
        self.distance_pub = rospy.Publisher(self.node_name + '/distance_graph', sensor_msgs.msg.Image)
        
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
            args = line.split(' ')
            handled = False
            successful = False

            if args[0] == 'run':
                # Register handling of command
                handled = True
                if len(args) > 1:
                    n = int(args[1])
                else:
                    n = 10
                run_result = self.run_demo(n)
                pickle.dump(run_result, open('last_result.pickle', 'wb'))
                
                
                
                # Register command handling successful
                successful = True
            
            
            
            if args[0] == 'dshow':
                # Register handling of command
                handled = True
                
                run_result = pickle.load(open('last_result.pickle'))
                self.plot_distance(run_result)
                
                # Register command handling successful
                successful = True
            
            if args[0] == 'goal':
                # Register handling of command
                handled = True
                
                self.set_goal()
                
                # Register command handling successful
                successful = True
                
            if args[0] == 'start':
                # Register handling of command
                handled = True
                
                self.set_current()
                
                # Register command handling successful
                successful = True
                
            if args[0] == 'show':
                # Register handling of command
                handled = True
                
                self.show_images()
                
                # Register command handling successful
                successful = True
                
            if args[0] == 'demo':
                handled = True
                
                
 
                if 'last' in args:
                    self.distance_grid = pickle.load(open('distance_grid.pickle'))
                    all_results = pickle.load(open('all_results.pickle'))                
                else:
                    self.track_state_home()
                    self.distance_grid = self.scan_environment_distances()
                    pickle.dump(self.distance_grid, open('distance_grid.pickle', 'wb'))
                    
                    all_results = self.demo_with_map()
                    pickle.dump(all_results, open('all_results.pickle', 'wb'))
                
#                pdb.set_trace()
                self.plot_distance(all_results[0])
                
                self.draw_map()
                self.draw_threshold()
                self.draw_results(all_results)
#                pylab.savefig('final.png')
                
                # Register command handling successful
                successful = True
                
            if args[0] == 'scan':
                # Register handling of command
                handled = True
                
                self.distance_grid = self.scan_environment_distances()
                self.draw_map()
                self.draw_threshold()
                pylab.savefig('test.png')
                
                # Register command handling successful
                successful = True
                
            if args[0] == 'call':
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
                    
            if args[0] == 'set':
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
        
    def run_demo(self, n):
#        args = line.split(' ')
#        if len(args) <= 1:
#            n = 10
#        else:
#            n = args[1]
        pylab.figure()
        iter_results = []
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
            
            iter_result = {'i':i,
                           'plan_found':tuple(plan),
                           'plan_executed':plan_executed,
                           'start_distance':dist0,
                           'final_distance':dist,
                           'predicted_distances':pred_dist,
                           'y_0':y_0,
                           'y_1':get_image_array(self.y_current)}
            iter_results.append(iter_result)
            
            
            image = self.update_distance_plot(iter_result)
#            ros_img = pil_to_imgmsg(image)
#            ros_img = numpy_to_imgmsg(np.array(image))
#            self.distance_pub.publish(ros_img)
            
        run_result = {'iter_results':iter_results}
        return run_result
        
    def plot_distance(self, run_result):
        '''
        
        :param run_result: is a dictionary with an iter_results list 
        containing results for each iteration in the planning.
        iter_result needs:
            - start_distance: (for first iteration)
            - final_distance: for all iterations
            - i: index of iteration
            - predicted_distances: distances for the predicted images from 
              found plan
        '''
        
        # Initiate new plot
        self.Y_vals = None
        pylab.figure()
        
        
        # Update the plot for each iter_result
        for iter_result in run_result['iter_results']:
            self.update_distance_plot(iter_result, annotate=False)
            
            
        # Initiate new plot
        self.Y_vals = None
        pylab.figure()
        
        
        # Update the plot for each iter_result
        for iter_result in run_result['iter_results']:
            self.update_distance_plot(iter_result, annotate=True)
            
    def update_distance_plot(self, iter_result, annotate=False):
        print('next iter')
        if not hasattr(self, 'Y_vals'):
            Y_vals = None
        else:
            Y_vals = self.Y_vals
        
        # Get the threshold for planning module
        try:
            thresh = float(rospy.get_param(self.server_name + '/planning_thresholds')[0])
        except:
            thresh = 0
            print('couldn\'t get threshold')
#        pdb.set_trace()
        # Process data
        i = iter_result['i']
        pred_dist = iter_result['predicted_distances']
        print('itter ' + str(i) + '   ' + str(pred_dist))
        dist0 = iter_result['start_distance']
        
        if Y_vals is None:
            Y_vals = [dist0]
        
        dist = iter_result['final_distance']
        
        Y_vals.append(dist)
        self.Y_vals = Y_vals
        x_vals = range(i, i + len(pred_dist) + 1)
#        pdb.set_trace()
        y_vals = (Y_vals[i],) + pred_dist
        
        print('ready to plot')
        
        # Define some artist styles for plotting
        l_pred_style = {'color':'gray',
                        'marker':'o',
                        'markersize':3}
        l_style = {'color':'b',
                   'linewidth':2,
                   'marker':'o',
                   'markersize':4}
        t_style = {'linestyle':'--'}
#        pdb.set_trace()
        # Plot data
        pylab.plot(x_vals, y_vals, **l_pred_style)
        pylab.plot(range(len(Y_vals)), Y_vals, **l_style)
        print('pass data')
        # Plot threshold
        pylab.axhline(thresh, **t_style)
        print('pass axhline')
        # Annotate plot
#        if not 'annotate' in locals():
#            annotate = False
        if not 'name' in locals():
            name = ''
             
        if annotate:
            pylab.legend(['Predicted', 'True', 'Threshold'],
                         loc=9, ncol=3)
            pylab.xlabel('Planning Iteration')
            pylab.ylabel('Distance to y_goal')
    
            pylab.savefig('dist_it' + str(i) + '.png')
        else:
            pylab.savefig('dist_it' + str(i) + '_noannotations.png')
        
        print('done with this plot')
        try:
            image_array = iter_result['y_1']
            Image.fromarray(image_array).save('y1_it' + str(i) + '.png')
        except:
            print('No image y1')
        return True
        
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
        pylab.figure()
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
        
#        starts = [(1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2),
#                  (1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0),
#                  (1, 1, 2),
#                  (0, 3, 3, 3),
#                  (2, 2, 3, 3, 3, 3)]
        starts = [(1, 1,0,0), (3,3,0,0), (0,0,0)]
        for start in starts:
            self.track_state_home()
            iter_results = []
            self.track_state_move(start)
            executed = []
            for i in range(20):
                state0 = self.state 
                dist0 = get_distance(0).res
                print('run_one ' + str(i))
                plan, plan_executed = self.run_one()
                if plan == (-1,):
                    break
                
                pred_dist = self.get_p_d(plan).array
                
                iter_results.append({'i':i,
                                     'plan_found':tuple(plan),
                                     'plan_executed':plan_executed,
                                     'start_state':state0,
                                     'start_distance':dist0,
                                     'final_distance':get_distance(0).res,
                                     'predicted_distances':pred_dist})
                
                time.sleep(1)
                self.track_state(plan_executed)
                for p in plan_executed:
                    executed.append(p)
            run_result = {'start_move': start,
                          'iter_results':iter_results,
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
            
            for i, iter_result in enumerate(run_result['iter_results']):
                
                # Plot the executed part
                self.plot_plan_line(iter_result['start_state'],
                               iter_result['plan_executed'],
                               **executed_style)
                pylab.annotate(str(i), xy=iter_result['start_state'], xytext=(5,5), textcoords='offset points')
                
                # Plot the found plan
                self.plot_plan_line(iter_result['start_state'],
                               iter_result['plan_found'],
                               **found_style)
                
                pylab.savefig('map_' + str(r) + '_it' + str(i) + '.png')
    
    def plot_plan_line(self, state0, plan, **lineargs):
        print('plot line')
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
