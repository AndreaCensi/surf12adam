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


# Initiate CV Britge for image manipulation in opencv
bridge = CvBridge()

def resize_image(image,size):
	# Convert image to cv image to zoom
	try:
		cv_image = bridge.imgmsg_to_cv(image, "bgr8")
	except CvBridgeError, e:
		print e
	cvi2 = cv.CreateImage(size,cv.IPL_DEPTH_8U,3)
	cv.Resize(cv_image,cvi2,interpolation=cv.CV_INTER_LINEAR)
	return bridge.cv_to_imgmsg(cvi2, "bgr8")

# Zoom an image with zoom center
def zoom_image(image,zoom):
	# Convert image to cv image to zoom
	try:
		cv_image = bridge.imgmsg_to_cv(image, "bgr8")
	except CvBridgeError, e:
		print e
		
	(cols,rows) = cv.GetSize(cv_image)
	if vars().has_key('centerx'):
		zcx = centerx
	else:
		zcx = cols/2
    	
	if vars().has_key('centery'):
		zcy = centery
	else:
		zcy = rows/2
	
	M = cv.CreateMat(2, 3, cv.CV_32FC1)
	M[0,0] = zoom/100.
	M[0,1] = 0
	M[0,2] = -zcx*(zoom/100.-1)
	M[1,0] = 0
	M[1,1] = zoom/100.
	M[1,2] = -zcy*(zoom/100.-1)
   
	cvi2 = cv_image;
	cv.WarpAffine(cv_image,cvi2,M)

	return bridge.cv_to_imgmsg(cvi2, "bgr8")


# compare_images
#
# Compares two images (/sensor_msgs/Image) and estimates first norm 
# of the difference between the pixel values by calculating the 
# average difference for the set of each <step>'th pixel.
# 
# Output
# 	Average pixel value difference
#   
def compare_images(image1, image2, step=100):
	normdiff = 0.
	n = len(image1.data)
	for i in range(0,n-1,step):
		normdiff += abs(int(image1.data[i].encode('hex'),16) - int(image2.data[i].encode('hex'),16))	
	return normdiff/n/step 
# returns the value of a pixel at position x,y in image image 
def get_pixel(image,x,y):
	width = image.width
	height = image.height
	pix_index = y*width + height
	return int(image.data[pix_index].encode('hex'),16)	
def get_area_average(image,x,y,w,h):
	width = image.width
	height = image.height
	sum = 0;
	for r in range(h-1):
		for c in range(w-1):
			sum += get_pixel(image,x+c,y+r)
	sum = sum/w/h
	return sum
	
# Script Start

# Initiate
print len(sys.argv)
for arg in sys.argv:
	print arg

try:
	infile = sys.argv[sys.argv.index('-i')+1]
except ValueError:
	print 'No input file specified'
	infile = '/home/adam/2012-07-06-17-50-39.bag'
	
try:
	outfile = sys.argv[sys.argv.index('-o')+1]
except ValueError:
	print 'No output file specified'
	outfile = 'outbag.bag'
	
try:
	width = sys.argv[sys.argv.index('-w')+1]
	height = sys.argv[sys.argv.index('-h')+1]
	output_size = [int(width), int(height)]
except ValueError:
	print 'No output file specified'
	output_size = (64,48)

# Save the last read image for use as Y0 when command is read
Y_last = 0

# Keep track of the actual zoom for the last command and the current zoom
zoom_last = 250
zoom = 100

out_bag = rosbag.Bag(outfile, 'w')

# Define state integers for image capturing states
state_takeY0 = 0			# Waiting for first image
state_wait_for_move = 1		# Wait for camera to start moving
state_cam_moving = 2		# State for when camera is moving
state_takeY1 = 3			# Waiting for last image

next_state = -1
state_capture = 0
diff_treshold = 0.001

move_waits = 0
move_timeout = 9

#bag = rosbag.Bag('/home/adam/2012-07-06-17-50-39.bag')
bag = rosbag.Bag(infile)
imglist = bag.read_messages('/usb_cam/image_raw')
diffsum = 0
numframes = 0
ic = 0
for topic, msg, t in imglist:
	if ic > 200:
		break
	if Y_last != 0:
		diff = compare_images(Y_last, msg)
		print diff
		diffsum += diff
		numframes += 1
	Y_last = msg
	ic += 1 
diff_treshold = diffsum/numframes*2
print 'diff_treshold = ', diff_treshold

list = bag.read_messages(topics=['/usb_cam/image_raw','/logitech_cam/camera_executed'])
i = 0
for topic, msg, t in list:
	if topic == '/logitech_cam/camera_executed':
		if Y_last != 0:
			print 'Camera Instruction Read: (',msg.data,')' 
			print 'Writing Y0 and U0'
			write_img = resize_image(zoom_image(Y_last,zoom_last),output_size)
			out_bag.write('Y0',write_img, t)
			out_bag.write('U0',msg,t)
			t0 = t
			if zoom_last + msg.data[2] >= 100:
				if zoom_last +msg.data[2] <= 500:
					zoom = zoom_last + msg.data[2]
			next_state = state_wait_for_move
			move_waits = 0
		
	if topic == '/usb_cam/image_raw':
		#print 'cam image read'
		if state_capture == state_wait_for_move:
			diff = compare_images(Y_last, msg)
			#print 'Waiting for move state, diff = ',diff
			if diff > diff_treshold:
				print 'Camera detectet to be moving'
				next_state = state_cam_moving
			move_waits +=1
			if move_waits > move_timeout:
				print 'Proceding to state_take_Y1, timeout in state_wait_for_move'
				next_state = state_takeY1
				
		if state_capture == state_cam_moving:
			diff = compare_images(Y_last, msg)
			#print 'Camera Moving State, diff = ', diff
			if diff < diff_treshold:
				print 'camera has stoped'
				next_state = state_takeY1
			move_waits +=1
			if move_waits > move_timeout:
				print 'Proceding to state_take_Y1, timeout in state_cam_moving'
				next_state = state_takeY1
				
		if state_capture == state_takeY1:
			print 'Writing Y1'
			#print output_size
			#print output_size.__class__
			write_img = resize_image(zoom_image(msg,zoom),output_size)
			out_bag.write('Y1',write_img,t0)
			i+=1
			
			next_state = state_takeY0
			
		Y_last = msg	# update last read image
		zoom_last = zoom
	state_capture = next_state
bag.close()
out_bag.close()
