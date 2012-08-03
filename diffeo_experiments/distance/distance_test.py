#!/usr/bin/env python
import roslib
roslib.load_manifest('logitech_cam')
import sys
import rosbag
import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import cv
from cv_bridge import CvBridge, CvBridgeError
import pdb
import ParseMessages
from distance import *
import matplotlib.pyplot as plt

def get_image_array(image):
    im,data,dimsizes = ParseMessages.imgmsg_to_pil(image)
    pix = np.asarray(im).astype(np.uint8)
    return pix

def load_processed_bag(infile):
    
    bag = rosbag.Bag(infile)
    list = bag.read_messages(topics=['Y0','Y1','U0'])
    Y0list = []
    Y1list = []
    U0list = []
    for topic, msg, t in list:
        if topic == 'Y0':
            Y0list.append(get_image_array(msg))
        if topic == 'Y1':
            Y1list.append(get_image_array(msg))
        if topic == 'U0':
            U0list.append(msg)
    return Y0list,Y1list,U0list
    
Y0list, Y1list, U0list = load_processed_bag('/media/data/processed-data/commands100t60.processed.bag')

#dist = distance.distance_neighborhood(Y0list[0],Y1list[0])
#pdb.set_trace()

distance_function_list = [distance.distance_scaled_mean, distance.distance_scaled_std,distance.distance_neighborhood_bestmatch,distance.distance_neighborhood_bestmatch,distance.distance_neighborhood_distance]
#distance_function_list = [distance.distance_neighborhood]
#opt_list = [1,1,[4,4],[6,6],[8,8]]
#distance.neighborarea_default = [7,7]
max_dept = 7
n_samp = 25

dist_mean_list = []
dist_std_list = []

plt.figure()
#for fi in range(len(distance_function_list)):

distance = distance()
distance.neighbor_indices_flat_init(Y0list[0],[6,6])
distance.calculate_max((160,120))
""" distance 5 """

#distance_function = distance_function_list[4]
#distance.neighbor_indices_flat_init(Y0list[0],[6,6])
#dist_list = [[]]*max_dept
#dist_mean = np.zeros(max_dept)
#dist_std = np.zeros(max_dept)
#for dept in range(max_dept):
#    print('Computing dept: '+str(dept))
#    dist = []
#    for i in range(n_samp):
#        if i+dept<len(Y0list):
#            y1 = Y0list[i]
#            y2 = Y0list[i+dept]
#            dist.append(distance.distance_neighborhood_distance(y1,y2))
#    dist_list[dept] = dist
#    dist_mean[dept] = np.mean(dist)
#    dist_std[dept] =  np.std(dist)
#dist_mean_list.append(dist_mean)
#dist_std_list.append(dist_std)
#print dist_mean
#print dist_std
#plt.plot(range(max_dept),dist_mean,color='c',label='distance_neighborhood_distance, area=[6,6]')
#plt.plot(range(max_dept),dist_mean+dist_std,'cs')
#plt.plot(range(max_dept),dist_mean-dist_std,'cs')

#""" distance 6 """
#
#distance_function = distance_function_list[4]
#distance.neighbor_indices_flat_init(Y0list[0],[6,6])
#dist_list = [[]]*max_dept
#dist_mean = np.zeros(max_dept)
#dist_std = np.zeros(max_dept)
#for dept in range(max_dept):
#    print('Computing dept: '+str(dept))
#    dist = []
#    for i in range(n_samp):
#        if i+dept<len(Y0list):
#            y1 = Y0list[i]
#            y2 = Y0list[i+dept]
#            dist.append(distance.distance_neighborhood_distance(y1,y2))
#    dist_list[dept] = dist
#    dist_mean[dept] = np.mean(dist)
#    dist_std[dept] =  np.std(dist)
#dist_mean_list.append(dist_mean)
#dist_std_list.append(dist_std)
#print dist_mean
#print dist_std
#plt.plot(range(max_dept),dist_mean,color='y',label='distance_neighborhood_distance, area=[6,6]')
#plt.plot(range(max_dept),dist_mean+dist_std,'ys')
#plt.plot(range(max_dept),dist_mean-dist_std,'ys')




""" distance 1 """
distance_function = distance_function_list[0]
dist_list = [[]]*max_dept
dist_mean = np.zeros(max_dept)
dist_std = np.zeros(max_dept)
for dept in range(max_dept):
    print('Computing dept: '+str(dept))
    dist = []
    for i in range(n_samp):
        if i+dept<len(Y0list):
            y1 = Y0list[i]
            y2 = Y0list[i+dept]
#            pdb.set_trace()
            dist.append(distance.distance_scaled_mean(y1,y2))
    dist_list[dept] = dist
    dist_mean[dept] = np.mean(dist)
    dist_std[dept] =  np.std(dist)
dist_mean_list.append(dist_mean)
dist_std_list.append(dist_std)
print dist_mean
print dist_std
plt.plot(range(max_dept),dist_mean,color='g',label='distance_scaled_mean, scale=1.0')
plt.plot(range(max_dept),dist_mean+dist_std,'gs')
plt.plot(range(max_dept),dist_mean-dist_std,'gs')


""" distance 2 """

distance_function = distance_function_list[1]
dist_list = [[]]*max_dept
dist_mean = np.zeros(max_dept)
dist_std = np.zeros(max_dept)
for dept in range(max_dept):
    print('Computing dept: '+str(dept))
    dist = []
    for i in range(n_samp):
        if i+dept<len(Y0list):
            y1 = Y0list[i]
            y2 = Y0list[i+dept]
            dist.append(distance.distance_scaled_std(y1,y2))
    dist_list[dept] = dist
    dist_mean[dept] = np.mean(dist)
    dist_std[dept] =  np.std(dist)
dist_mean_list.append(dist_mean)
dist_std_list.append(dist_std)
print dist_mean
print dist_std
plt.plot(range(max_dept),dist_mean,color='b',label='distance_scaled_std, scale=1.0')
plt.plot(range(max_dept),dist_mean+dist_std,'bs')
plt.plot(range(max_dept),dist_mean-dist_std,'bs')


""" distance 3 """

distance_function = distance_function_list[2]
distance.neighbor_indices_flat_init(Y0list[0],[4,4])
dist_list = [[]]*max_dept
dist_mean = np.zeros(max_dept)
dist_std = np.zeros(max_dept)
for dept in range(max_dept):
    print('Computing dept: '+str(dept))
    dist = []
    for i in range(n_samp):
        if i+dept<len(Y0list):
            y1 = Y0list[i]
            y2 = Y0list[i+dept]
            dist.append(distance.distance_neighborhood_bestmatch(y1,y2))
    dist_list[dept] = dist
    dist_mean[dept] = np.mean(dist)
    dist_std[dept] =  np.std(dist)
dist_mean_list.append(dist_mean)
dist_std_list.append(dist_std)
print dist_mean
print dist_std
plt.plot(range(max_dept),dist_mean,color='r',label='distance_neighborhood_bestmatch, area=[4,4]')
plt.plot(range(max_dept),dist_mean+dist_std,'rs')
plt.plot(range(max_dept),dist_mean-dist_std,'rs')


""" distance 4 """

distance_function = distance_function_list[3]
distance.neighbor_indices_flat_init(Y0list[0],[6,6])
dist_list = [[]]*max_dept
dist_mean = np.zeros(max_dept)
dist_std = np.zeros(max_dept)
for dept in range(max_dept):
    print('Computing dept: '+str(dept))
    dist = []
    for i in range(n_samp):
        if i+dept<len(Y0list):
            y1 = Y0list[i]
            y2 = Y0list[i+dept]
            dist.append(distance.distance_neighborhood_bestmatch(y1,y2))
    dist_list[dept] = dist
    dist_mean[dept] = np.mean(dist)
    dist_std[dept] =  np.std(dist)
dist_mean_list.append(dist_mean)
dist_std_list.append(dist_std)
print dist_mean
print dist_std
plt.plot(range(max_dept),dist_mean,color='m',label='distance_neighborhood_bestmatch, area=[6,6]')
plt.plot(range(max_dept),dist_mean+dist_std,'ms')
plt.plot(range(max_dept),dist_mean-dist_std,'ms')


#plt.legend(loc='best')


plt.savefig('distande-plot.png')

pdb.set_trace()