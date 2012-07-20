#!/usr/bin/env python
import roslib 
import sys
roslib.load_manifest('logitech_cam')
import rospy
import random
from std_msgs.msg import String
from logitech_cam.msg import IntArray


def parse_commands(str):
	cmd_strs = str.strip('[]').split('],[')
	num_cmd = len(cmd_strs)
	
	cmd_list = [[]]*num_cmd
	
	for i in range(num_cmd):
		cmd_list[i] = map(int,cmd_strs[i].split(','))
	
	print 'Command List Parsed:'
	print '============================='
	print 'Found ',num_cmd,' commands:'
	print cmd_list
	return cmd_list

# Generates a IntArray message with the command data given in intarray
def get_msg(intarray):
	msg = IntArray()
	msg.data = intarray
	return msg
	

def main(args):
	rospy.init_node('command_generator', anonymous=True)
	#print args
	# Test string for commands
#	if rospy.has_param('/command_generator/logitech_cam/command_list'):
#		cmd_str = rospy.get_param('/command_generator/command_list')
#	else:
	# Check for arguments
	try:
		cmd_str = args[args.index('-c')+1]
	except ValueError:
		print 'No given commands found, using default command string' 
		cmd_str = '[[150,0,0],[-150,0,0],[0,100,0],[0,-100,0],[0,0,10],[0,0,-10]]'
	
	try:
		duration = int(args[args.index('-t')+1])
		print 'Running for ',duration,'s'
	except ValueError:
		print 'No duration found, running for 60s.'
		duration = 60
	
	# Generation frequency
	try:
		freq = float(args[args.index('-f')+1])
	except ValueError:
		freq = 1.0
	print freq
	
	
	# Start at random position ?
	try:
		start_random = args[args.index('-r')+1].lower() in ("yes", "true", "t", "1")
	except ValueError:
		start_random = False
	print 'start random position: 	',start_random
	
	time0 = rospy.get_time()
	
	cmd_list = parse_commands(cmd_str)
	num_cmd = len(cmd_list)
	cmd_publisher = rospy.Publisher('/logitech_cam/camera_instr',IntArray)
	
	# Initiate random number generator  
	random.seed()
	
	# Set the frequence for sending commands
	### make rate as parameter
	r = rospy.Rate(freq)
	while not rospy.is_shutdown():
		while rospy.get_time()-time0 < duration:
			ri = random.randint(0,num_cmd-1)
			print rospy.get_time()-time0
			this_cmd = cmd_list[ri]
			print 'Sending command: 	',this_cmd
			cmd_publisher.publish(get_msg(this_cmd))
			r.sleep()
		
	print 'Shutting down'

if __name__ == '__main__':
	main(sys.argv)
