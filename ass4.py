#!/usr/bin/env python


import rospy
import actionlib
from get_command_from_user import distance_to_colored_object_helper,turn_around_helper,move_forward 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
#from node1 import distance_helper
from sensor_msgs.msg import LaserScan
 
 
### bypass the obstacle from the right side , then turn left to push it
def move_to_right(counter): 
    
    move_forward(0.35,True)
    rospy.sleep(1.)
    turn_around_helper(-150+counter,True)
    print "turned 150"
    move_forward(0.45,False)
    rospy.sleep(1.)
    move_forward(-0.20,False)
    turn_around_helper(90,True)


### bypass the obstacle from the right side, then turn right to push it
def move_to_left(counter): 
    
    move_forward(0.35,True)
    rospy.sleep(1.)
    turn_around_helper(150-counter,True)
    print "turned 150"
    move_forward(0.45,False)
    rospy.sleep(1.)
    move_forward(-0.20,False)
    turn_around_helper(-90,True)
    

 ### in this function, we try to by pass the blue object from one of the sides, then push it
 ### param - counter - this param affect how much the robot would turn in order to bypass the blue object. 
 ###					We call this function at first with counter = 0 - which means we would turn 60 degrees each side. 
 ###					If it can't pass, we would call this function in recursion with an increasing counter - causing the robot to turn less.   
def check_sides_and_turn(counter):  

    ### checking right side first
    turn_around_helper(60-counter,True)
    rospy.sleep(1.)
    laserInfo = rospy.wait_for_message('/scan',LaserScan)
    right_side = True
    
    ### checking there's enough place for the robot to pass
    for i in range(17):
        print "in degree %s the dist is %s" %( i,laserInfo.ranges[i])
        if laserInfo.ranges[i] < 0.3 and not laserInfo.ranges[i]==0:
            right_side  = False 
    for i in range(343, 359):
        print "in degree %s the dist is %s" %( i,laserInfo.ranges[i])
        if laserInfo.ranges[i] < 0.3 and not laserInfo.ranges[i]==0:
            right_side  = False  
    if right_side:
        move_to_right(counter)
    
    
    else:
        ### checking the left side!!
        turn_around_helper(-120+2*counter,True) 
        rospy.sleep(1.)
        laserInfo = rospy.wait_for_message('/scan',LaserScan)
        left_side = True
        ### checking there's enough place for the robot to pass
        for i in range(17):
            print "in degree %s the dist is %s" %( i,laserInfo.ranges[i])
            if laserInfo.ranges[i] < 0.3 and not laserInfo.ranges[i]==0:
                left_side  = False 
        for i in range(343, 359):
            print "in degree %s the dist is %s" %( i,laserInfo.ranges[i])
            if laserInfo.ranges[i] < 0.3 and not laserInfo.ranges[i]==0:
                left_side  = False  
                
        ### left_side = false means he can't move in both sides, so we'll try          
        if not left_side and counter <8: # we limit the the degree
            print "could not move both ways"
            turn_around_helper(60+counter,True)
            check_sides_and_turn(counter+4)
            
        else :
            move_to_left(counter)
		
    
    
 
def moveBlueObject1(dist):

    print "before getting closer to the blue , the dist is %s" % dist
    move_forward(dist-0.22,True) # getting closer to the blue object by 0.2 m
    rospy.sleep(1.)
    print "finished moving forward"
    

    check_sides_and_turn(0)
    
    
    
    
def movebase_client(xgoal,ygoal):

    ### turning around 360 degrees so the robot woud understand where he is 
    turn_around_helper(90,True)
    turn_around_helper(90,True)
    turn_around_helper(90,True)
    turn_around_helper(90,True)
    rospy.sleep(1.)
    
    
    ### using MoveBaseGoal class - this class gets the get goal parametes and navigate to it, while avoiding obstacles
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = xgoal
    goal.target_pose.pose.position.y = ygoal
    goal.target_pose.pose.orientation.w = 1
    
    client.send_goal(goal)
    
    ### checking if he sees the blue obstacle
    dist = distance_to_colored_object_helper("blue",False)
    
    
    ### stoppping when getting closer to the blue by 0.5 m
    while dist is None or dist > 0.55:
        dist = distance_to_colored_object_helper("blue",False)
    print("last distance here is %s" % dist)
   
    client.cancel_goal() ### cancelling the navigation, so we woould push away the obstacle
    print("goal has been canceled")
    
    
    dist = distance_to_colored_object_helper("blue",True)  ## getting the exact distance from the object
    x=1
    sign = 1
    while dist==None or dist==0: ### sometimes it failes to calculate the right distance, so we turn the robot a bit and try again  
        print "dist is none or 0 , trying again"
        turn_around_helper(2*x*sign, True)
        sign = sign * -1
        x=2*x
        dist = distance_to_colored_object_helper("blue",True)
    print("last distance  0 degrees here is %s" % dist)
    
    rospy.sleep(1.)
    print ("woke up")
  
    
    # Time to move the obstacle!!
    moveBlueObject1(dist) 
    
   
    #### After moving the obstacle we resume the navigation to the end
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        
        ## The Goal Point
        xgoal = 2.59499955177 
        ygoal = 0.469999730587
        
        rospy.init_node('movebase_client_py')
        result = movebase_client(xgoal,ygoal)
        
        
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
