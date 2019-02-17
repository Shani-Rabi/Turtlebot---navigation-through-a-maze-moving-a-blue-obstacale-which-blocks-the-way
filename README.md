# Turtlebot---navigation-through-a-maze-moving-a-blue-obstacale-which-blocks-the-way
Robotics Final project 



Assignment Definition:
The final project involves navigating in an obstacle course. A practice course is already set up in the lab. You need to move from one end to the other, and in the process, recognize the blue box. Your robot has to push this box aside in order to continue to the exit of the course.


How to Run:
1. Open Rviz, set the robot position to the goal point. Run in terminal: rostopic echo /amcl_pose. Change xgoal, ygoal parametrs in the main
according to the this point.
2. Set the robot position back to the start of the maze.
3. run the python code (The main file is called ass4.py). 


Algorithm Idea:

1. The robot would turn 360 degrees in order for it to identify better where he is in the map.
2. The robot would start navigate to the end of the maze, using "move_base" package. 
3. The robot would stop navigating when it identifies the blue box in less then 0.55 m.
4. The robot would turn to the center of the blue obstacle and approach it by ~20 cm.
5. The robot would turn 60 degrees to the right, then would check the +-17 degrees in front of him using the laser - if there's
   enough place (0.3 cm) for the robot to move forward - Do so. Otherwise, turn 60 degress back + 60 degrees to the left and check the
   second side this time.
   If there's no place in both side, try turning less then 60 degrees (tries 3 different degrees).
6. After moving forward, turn around to face the blue box - then move forward and push it. 
7. The robot would move backward a bit, then resume the navigation to the end.
   

   
In each step we care not to crash into the walls - in stage 2 the move_base package take care of it for us. In stages 5 and 6 we stop when getting close
to the walls by ~10 cm.
