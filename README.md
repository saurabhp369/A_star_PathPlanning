Requirements:

1) Python 3.8 (Since we are using math.dist() for eucledian distance which is available in Python >=3.8)

2) OpenCV (cv2)

3) math

4) numpy

5) matplotlib

Instructions:

Run the python file by "python3 A_star.py"

Enter the user inputs when prompted. (If prompted to re-enter after entering all inputs, then please re-enter some valid coordinates which are not in any obstacles)

In the user inputs, steps of 30 in each direction means defines the action set. For example, if you enter 2, the actionset will be [-60, -30, 0, 30, 60]. If you enter 3, the actionset would be  [-90, -60, -30, 0, 30, 60, 90] and so on.
	
Once done, you should see "Goal reached" in the terminal with total cost printed. 
	
Now, you should see a video file by the name "Astar_visualisation.mp4" in the same directory as the python file. This would be the output showing explored states in yellow and the path in black once done coloring all explored nodes. Obstacles are in red/brown color.
	
Obstacle space:

<img src="https://github.com/saurabhp369/A_star_PathPlanning/blob/main/Visualisation/Obstacle_space.png">
	
Sample output is shown as follows:

<img src="https://github.com/saurabhp369/A_star_PathPlanning/blob/main/Visualisation/A_star.png" width = "400" height = "200">

Video link: https://github.com/saurabhp369/A_star_PathPlanning/blob/main/Visualisation/Astar_visualisation.mp4

