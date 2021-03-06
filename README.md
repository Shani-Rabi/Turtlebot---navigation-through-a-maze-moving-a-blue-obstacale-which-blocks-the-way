# Turtlebot---navigation-through-a-maze-moving-a-blue-obstacale-which-blocks-the-way
Robotics Final project 



Assignment Definition:
The final project involves navigating in an obstacle course. A practice course is already set up in the lab. You need to move from one end to the other, and in the process, recognize the blue box. Your robot has to push this box aside in order to continue to the exit of the course.


How to Run:
1. Update your parameters according to the maze. We've attached ours.
2. Open Rviz, set the robot position to the goal point. Run in terminal: rostopic echo /amcl_pose. Change xgoal, ygoal parametrs in the main
according to the this point.
3. Set the robot position back to the start of the maze.
4. run the python code (The main file is called ass4.py). 



Algorithm Idea:

1. The robot turns 360 degrees in order to better understand its location in the map.
2. The robot starts to navigate to the end of the maze, using "move_base" package. 
3. The robot stops navigating when it identifies the blue box in less then 0.55 m. In order to find the blue box the robot uses 
   the usb cam and the laser: we define the BGR range for the blue color, and turn the image to black and white (blue turns to white,
   any other thing to black). We look for contours in this gray image - if there's a countour big enough (compared to the
   image size), we turn the robot to be in front of it, and use the laser to find the distance to it. 
4. The robot turns to the center of the blue obstacle and approaches it by ~20 cm.
5. The robot turns 60 degrees to the right, then checks the +-17 degrees in front of him using the laser - if there's
   enough place (0.3 cm) for the robot to move forward - Do so. Otherwise, turn 60 degress back + 60 degrees to the left and check the
   second side this time.
   If there's no place in both sides, try turning less then 60 degrees (tries 3 different degrees).
6. After moving forward, the robot turns around to face the blue box - then moves forward and pushes it. 
7. The robot moves backward a bit, then resumes navigation to the end.
   

   
In each step we pay attention to the walls to avoid crashing - in stage 2 the move_base package takes care of it for us. In stages 5 and 6 we stop when getting closer than ~10 cm to the walls.
