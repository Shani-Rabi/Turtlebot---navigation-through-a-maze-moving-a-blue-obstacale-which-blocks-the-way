#!/usr/bin/env python


from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Quaternion
from image_geometry import PinholeCameraModel
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from geometry_msgs.msg import Twist
import math 
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
roll = pitch = curr_yaw = axes = 0
first_yaw_radians = "start"
from sensor_msgs.msg import LaserScan
going_forward = True
from sys import exit
can_move = "not initialized"
x_curr = "not initialized"
y_curr = 0
x_first = "not initialized"
y_first = 0
found_obj = "not initialized"
ranges = 0 

	
	
def check_obstacles(msg): 
	global can_move, ranges	
	ranges = msg.ranges
	
	scan_filter = []
        for i in range(360):
            if i <= 15 or i > 335:
                if msg.ranges[i] >= 0.01:
                    scan_filter.append(msg.ranges[i])

        if len(scan_filter)>0 and min(scan_filter) > 0.19:
            can_move = True
        else:
             can_move = False
             print "can't move"
             

		
def get_position(msg):
	global x_curr,y_curr, x_first, y_first
	position = msg.pose.pose.position
	if x_first == "not initialized":
		x_first = position.x   
		y_first = position.y 	
	else:
		x_curr = position.x
		y_curr = position.y

def move_forward(distance,checkObs):
	global x_curr,y_curr, x_first, y_first
	global can_move, ranges
	backword = 1
	x_curr = x_first = can_move = "not initialized"
	if checkObs: ### this params means we want the robot to move forward, not considering obstacles (used for example when we want to pushs the box)
            sub_helper_laser = rospy.Subscriber('/scan', LaserScan, check_obstacles)
        else:
            can_move = True
	sub_helper_pose = rospy.Subscriber('/odom', Odometry, get_position)

	#rospy.loginfo("Robot starting to move 50cm forward if he can")
	publisher_handler = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	r = rospy.Rate(10)
	move_parametes = Twist()
	

    current_distance = 0
	if (distance < 0):
            distance = -1 * distance
            backword = -1
	
	while can_move == "not initialized" or x_first == "not initialized" or x_curr == "not initialized" :
		pass
	
	
	if can_move or not checkObs:
            
		while (not rospy.is_shutdown()) and current_distance < distance and (can_move or not checkObs): # we will move till we pass 0.5 m
			
			move_parametes.linear.x = backword * 0.05  
			move_parametes.angular.z = 0	 # no angle
			publisher_handler.publish(move_parametes)
			r.sleep()			
			# distance between 2 points, by fomula			
			current_distance = math.sqrt(math.pow((x_first - x_curr),2) + math.pow( (y_first - y_curr), 2) )
		

		move_parametes.linear.x = 0
		#Force the robot to stop
		publisher_handler.publish(move_parametes)

	else:
		print ("There's an obstacle, robot can't move")

def get_rotation (msg):
	global roll, pitch, curr_yaw, first_yaw_radians 
	orientation_q = msg.pose.pose.orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	if first_yaw_radians == "start":
		roll, pitch, first_yaw_radians = euler_from_quaternion (orientation_list)		
	else:
		roll, pitch, curr_yaw = euler_from_quaternion (orientation_list)
    	

	
	
def turn_around_helper(degrees_to_add, do_it_the_short_way):
	print " in turn_around_helper" 	
	global roll, pitch, curr_yaw, first_yaw_radians 
	first_yaw_radians = curr_yaw = "start"
	sub_handler = rospy.Subscriber('/odom', Odometry, get_rotation)
	while first_yaw_radians == "start" or curr_yaw == "start" :
		pass
        first_yaw_degrees = 180 * first_yaw_radians / math.pi

	# calculating dest degrees (range is -180 to 180)  
	if (first_yaw_degrees < 0):
		first_yaw_degrees = (first_yaw_degrees+360)

	destination_degrees = first_yaw_degrees - degrees_to_add # we want to move clockwise - opposite to the real degrees 
	while destination_degrees< 0:
		destination_degrees = destination_degrees + 360
	while destination_degrees > 360:
		destination_degrees = destination_degrees - 360
	
	if (destination_degrees>180):
		destination_degrees = destination_degrees - 360
	destination_radians = math.pi * destination_degrees / 180
	
        
	turn_around_parametes = Twist()  
	publisher_handler = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	r = rospy.Rate(10);
	#rospy.loginfo("Robot starting to turn around")
	
	sign = -1 # define if the robot would turn clock wise
	
	if (do_it_the_short_way):
		if (destination_radians > 0 and curr_yaw > 0 and destination_radians > curr_yaw ):
			sign = 1  # the robot turn clockwise when the value is negative.
		else:
			if (destination_radians > 0 and destination_radians < 1.5  and curr_yaw < 0 ):
				sign = 1
			else:
				if (destination_radians < -1.5  and curr_yaw > 0 and destination_radians > -3 ):
					sign = 1
				else:
					if (destination_radians < 0  and curr_yaw < 0 and destination_radians > curr_yaw ):
						sign = 1
        print " before whie"
	while not rospy.is_shutdown():			
		turn_around_parametes.angular.z = (0.3 * sign)	
		publisher_handler.publish(turn_around_parametes) 
		r.sleep()
		#print "destination_radians is %s" %destination_radians
		#print "curr_yaw is %s" %curr_yaw
		if abs(destination_radians - curr_yaw) <0.04:
			turn_around_parametes.angular.z = 0 # we stop the robot
			publisher_handler.publish(turn_around_parametes)  	
			break
	
        

def calc_distance(degrees) :
        
	laserInfo = rospy.wait_for_message('/scan',LaserScan)

        
	distance = np.inf
	if not laserInfo.ranges[degrees] == np.inf :
		return laserInfo.ranges[degrees]
	      
    return distance
    
                    
def distance_to_colored_object_helper(color,needToTurn) :
        print " in distance_to_colored_object_helper"
        global found_obj 
        camInfo = rospy.wait_for_message('/usb_cam/camera_info',CameraInfo)
        IMAGE = rospy.wait_for_message('/usb_cam/image_raw',Image)
        camera = PinholeCameraModel()
        camera.fromCameraInfo(camInfo)
	bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(IMAGE,'bgr8')
        
    
        
        gau_blur = cv2.GaussianBlur(cv_image, (3,3), 0)
        #cv2.imshow('image',cv_image)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
        if (color == 'blue'): 
            lower = np.array([135,115,80], dtype="uint8")
            upper = np.array([255,160,120 ], dtype="uint8")
        elif (color == 'red'):
            lower = np.array([0,25,110], dtype="uint8")
            upper = np.array([50,100,255], dtype="uint8")
        elif (color == 'green'):
            lower = np.array([75,120,80], dtype="uint8")
            upper = np.array([110,255,110], dtype="uint8")
        else:
            print("invalid color. exit command")
            exit()
        mask_image = cv2.inRange(gau_blur, lower, upper)

        kernelOpen = np.ones((5,5))
        kernelClose = np.ones((20,20))
        maskOpen = cv2.morphologyEx (mask_image ,cv2.MORPH_OPEN,kernelOpen)
        final_mask = cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)	        
	
        
        
        
        #find the center of object
        img2, contours, h = cv2.findContours(final_mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        contour_area = False
        for i in range (len(contours)) :
            print "area is %s" % cv2.contourArea(contours[i])
            if (cv2.contourArea(contours[i]) > 40000 ):  # we want to consider only big enough blue objects 
                contour_area = True
     
        #rospy.sleep(10.)
        #cv2.imshow('countour',img2)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
	if(len(contours)!=0 and contour_area):
                print("contour is not 0")
                found_obj = True
                cnt = contours[0]
                for i in range (len(contours)) :
                    if cv2.contourArea(contours[i]) > cv2.contourArea(cnt): # we take the biggest one
                        cnt = contours[i]
		
        	M = cv2.moments(cnt)
        	x_center_obj = int(M['m10']/M['m00'])		
        	y_center_obj = 240 
		center_obj = (x_center_obj,y_center_obj)
		image_center = (320, 240) 
		ray_obj = camera.projectPixelTo3dRay(center_obj)
		ray_img = camera.projectPixelTo3dRay(image_center)
		 
		# angle between two vectors - by formula. We'll get the cos of the angle so we'll do arccos
	
		radians = math.acos ( np.dot (ray_obj, ray_img) / ( np.linalg.norm(ray_obj) * np.linalg.norm(ray_img) ) ) 
		degrees = int(math.degrees(radians))
		#if not needToTurn:
                if abs(x_center_obj-image_center[0])>100:
                    degrees = degrees + 5
                    degrees = degrees%360
                elif abs(x_center_obj-image_center[0])>40 :
                    degrees = degrees + 2
                    degrees = degrees%360
                
                if x_center_obj>= image_center[0]:
                    degrees = 360-degrees
                if degrees == 360:
                    degrees = 0
                
               
                if needToTurn:
                    print "xcenter - image_center = %s" %(x_center_obj-image_center[0])
                    cv2.imshow('image',cv_image)
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()
                    cv2.imshow('countour',img2)
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()
                    laserInfo = rospy.wait_for_message('/scan',LaserScan)
                    print "degree sent to turn is %s" %degrees
                    
                    turn_around_helper(-degrees, True) #turn the robot
                    degrees = 0
                    
                    
                    if laserInfo.ranges[0] > 0.55:
                        laserInfo = rospy.wait_for_message('/scan',LaserScan)
                        print "ERROR!! the robot is close but got wrong distance in degree 0"
                        for x in range(0, 360):
                            print " in degeree %s the dist is %s" % (x, laserInfo.ranges[x])
                        #rospy.sleep(10)
                
		
		distance = calc_distance(degrees)
		print " Distance to object is %s from degree %s " % (distance, degrees)
		if distance == 0:
                    print " dist is 0!!!!!"
                    return 0
                    #cv2.imshow('countour',img2)
                    #scv2.waitKey(0)
                    #cv2.destroyAllWindows()
                return distance
	else:
                found_obj = False
		print(color + " object doesn't exist. exit command.")
                return None

  	
