#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import math
import cv2

########################### Authors #####################
# 117054859 Aditya Varadaraj
# 118133959 Saurabh Palande
#########################################################

# Generalized Action Function        
def take_action(cur_state,l, i):
    status = False
    x = cur_state[0] + l*math.cos(math.radians(cur_state[2]+(i*30)))
    y = cur_state[1] + l*math.sin(math.radians(cur_state[2]+(i*30)))
    if (y <= 250 and y>=0 and x>=0 and  x<= 400):
        new_node = [x, y, int(cur_state[2]+(i*30))]
        n = new_node.copy()

        if n[2]<0:
            n[2] = (360 + n[2])
        elif n[2]>360:
            n[2] = (n[2]-360)
        else:
            n[2] = n[2]
        status = True
    else:
        n  = [0,0,0]
    return status, n

def euclidean_dist(p1,p2):
    point1 = [p1[0],p1[1]]
    point2 = [p2[0],p2[1]]
    return round(math.dist(point1,point2),2)

# function to check if the goal has reached
def check_goal(g, node):
    n = node.copy()
    d = euclidean_dist(g,n)

    t = abs(g[2] - n[2])
    return(d<=1.5 and t<=30)

#function to check if the child node is in obstacle space or not
def check_obstacle(node,c,r):
    a = math.sqrt(3)
    x = node[0]
    y = node[1]
    flag1 = False
    flag2 = False
    flag3 = False
    flag4 = False
        # Polygon space with clearance + radius
    if (x>=36-c-r and x<=115+c+r and y>=100-c-r and y<= 210+c+r ):
        if((y - (6/7)*x >= 780/7-c-r) or (y + (16/5)*x <= 2180/5+c+r)):
            if((y + (85/69)*x >= 15825/69-c-r) and (y -(25/79)*x <=13320/79+c+r)):
                flag1 = True

    # Circle with clearance + radius
    if((x-300)**2 + (y-185)**2 <= pow((40+c+r), 2)):
        flag2 = True

    # Hexagon with clearance + radius
    if ( x >= 165-c-r and x <= 235+c+r and y>= (100 - 70/a)-c-r and y<= (100 + 70/a)+c+r):
        if ((y + (2021/3500)*x >= 61273/350 -c-r) and (y - (2021/3500)*x >= -19567/350 - c-r)):
            if ((y + (101/175)*x <= 179087/700 +c+r) and (y - (101/175)*x <= 17487/700 + c+r)):
                flag3 = True

    # padding of clearance+radius on both x and y
    if(((x>=0) and (x<=5)) or ((x>=395) and (x<=400)) and ((y>=0) and (y<=5)) or ((y>=245) and (y<=250))):
        flag4 = True
    
    return (flag1 or flag2 or flag3 or flag4)

#function to check if the child node is in visited or not
def check_visited(node, v_array):
    n = node.copy()
    for i in range(2):
        
        x = n[i]%(math.floor(n[i]))
        if(x<0.25):
            n[i] = math.floor(n[i])
        elif(x > 0.75):
            n[i] = math.floor(n[i]) + 1
        else:
            n[i] = math.floor(n[i])+0.5

    n[2] = n[2]/30
    return v_array[int(2*n[1]), int(2*n[0]), int(n[2])-1] == 1

# function to find the parent node (used in back-tracking)
def find_parent(c,visited_list):
    for i in range(len(visited_list)):
        if(visited_list[i][2] == c):
            return i
            break

def create_obstacles(r,c):
    
    grid_points =[]
    obstacles=[]
    obstacles_clearance = []
    x= 400
    y = 250
    a = math.sqrt(3)
    for i in range(x+1):
        for j in range(y+1):
            grid_points.append([i,j])
    # define the obstacle space
    for x,y in grid_points:
    # for the polygon
        if (x>=36 and x<=115 and y>=100 and y<= 210 ):
            if((y - (6/7)*x >= 780/7) or (y + (16/5)*x <= 2180/5 )):
                if((y + (85/69)*x >= 15825/69) and (y -(25/79)*x <=13320/79)):
                    obstacles.append([x,y])
        # Polygon space with clearance of c and robot radius of r
        if (x>=36-c-r and x<=115+c+r and y>=100-c-r and y<= 210+c+r ):
            if((y - (6/7)*x >= 780/7-c-r) or (y + (16/5)*x <= 2180/5+c+r)):
                if((y + (85/69)*x >= 15825/69-c-r) and (y -(25/79)*x <=13320/79+c+r)):
                    obstacles_clearance.append([x,y])

        # for the circle
        if((x-300)**2 + (y-185)**2 <= 1600):
            obstacles.append([x,y])
        # Circle with clearance of c and robot radius of r
        if((x-300)**2 + (y-185)**2 <= pow((40+c+r), 2)):
            obstacles_clearance.append([x,y])

        # for the hexagon
        if ( x >= 165 and x <= 235 and y>= (100 - 70/a) and y<= (100 + 70/a)):
            if ((y + (2021/3500)*x >= 61273/350) and (y - (2021/3500)*x >= -19567/350 )):
                if ((y + (101/175)*x <= 179087/700) and (y - (101/175)*x <= 17487/700 )):
                    obstacles.append([x,y])
        # Hexagon with clearance of c and robot radius of r
        if ( x >= 165-c-r and x <= 235+c+r and y>= (100 - 70/a)-c-r and y<= (100 + 70/a)+c+r):
            if ((y + (2021/3500)*x >= 61273/350 -c-r) and (y - (2021/3500)*x >= -19567/350 - c-r)):
                if ((y + (101/175)*x <= 179087/700 +c+r) and (y - (101/175)*x <= 17487/700 + c+r)):
                    obstacles_clearance.append([x,y])

        # padding of 5 on both x and y
        if(((x>=0) and (x<=5)) or ((x>=395) and (x<=400))):
            obstacles_clearance.append([x,y])
        if(((y>=0) and (y<=5)) or ((y>=245) and (y<=250))):
            obstacles_clearance.append([x,y])

    o = np.array(obstacles)
    oc = np.array(obstacles_clearance)
    plt.xlim(0, 400)
    plt.ylim(0, 250)
    plt.scatter(oc[:,0], oc[:,1], c = 'g', s= 1, label = 'clearance and robot radius')
    plt.scatter(o[:,0], o[:,1], c = 'r', s =1, label = 'obstacles')
    plt.title('Obstacle space')
    plt.savefig('Obstacle_space.png')
    
    return obstacles, obstacles_clearance

def modify_visited_array(array, node):
    a = array.copy()
    n = node.copy()
    for i in range(2):
        x = node[i]%(math.floor(n[i]))
        if(x<0.25):
            n[i] = math.floor(n[i])
        elif(x > 0.75):
            n[i] = math.floor(n[i]) + 1
        else:
            n[i] = math.floor(n[i])+0.5
    n[2] = n[2]/30
    a[int(2*n[1]),int(2*n[0]), int(n[2])-1] =1
    return a

def A_star(open_queue, start, goal, obstacles_clearance, l, k, o, b, c, r):
    print('*******Started A* Algorithm*******')
    visited_array = np.zeros((501,801,12))
    visited_list = []
    node_index = 1 # initial node index
    parent_node = 1 #initial parent index
    ctc = 0
    ctg = euclidean_dist(start, goal)
    tc = ctc + ctg
    open_queue.update({tuple(start):[tc, ctc, node_index ,parent_node]})# adding start to the open list
    while True:
        n_k = min(open_queue.items(), key=lambda x: x[1][0])[0]
        n_v = open_queue[n_k]
        del open_queue[n_k]
        cur_node = n_k
        visited_list.append([n_v[0], n_v[1], n_v[2], n_v[3], n_k])
        visited_array = modify_visited_array(visited_array, list(cur_node))
        if(check_goal(goal, list(cur_node))):
            print('*******Goal reached*******')
            print('The total cost to reach the goal is', n_v[0])
            break
        else:
            for i in range(-k,k+1):
                status, node_state = take_action(list(cur_node), l, i)
                if status:
                    if((not check_obstacle(node_state, c, r)) and (not check_visited(node_state, visited_array))):
                        cv2.line(b, (int(2*n_k[0]), int(500 - 2*n_k[1])), (int(2*node_state[0]), int(500-2*node_state[1])),(0,255,255), 1 )
                        ctc = n_v[1] + 1
                        ctg = euclidean_dist(goal, node_state)
                        tc = ctc + ctg
                        if (tuple(node_state) in open_queue.keys()):
                            val = open_queue[tuple(node_state)][0]
                           
                            if(val > tc):
                                open_queue[tuple(node_state)] = [tc,ctc, open_queue[tuple(node_state)][2], n_v[2]]
                               
                        else:
                            node_index +=1
                            open_queue[tuple(node_state)] = [tc,ctc, node_index, n_v[2]]
                           
                o.write(b)
           
    return visited_list, o, b, n_v[0]

def main():
    node_queue = {} #open list
    correct = False
    while(not correct):
        print('Enter the start and goal location coordinates(x,y,theta)')
        s_x = int(input('Enter the x coordinate of start location '))
        s_y = int(input('Enter the y coordinate of start location '))
        theta_s = int(input('Enter the start angle'))
        g_x = int(input('Enter the x coordinate of goal location '))
        g_y = int(input('Enter the y coordinate of goal location '))
        theta_g = int(input('Enter the goal angle'))
        l = int(input('Enter the step size (between 1 and 10)'))
        k = int(input('Enter the number of steps of 30 in each direction'))
        r = int(input('Enter the robot radius'))
        cl = int(input('Enter the clearance'))
        start = [int(s_x), int(s_y), theta_s]
        goal = [int(g_x), int(g_y), theta_g]
        obstacles, obstacles_clearance = create_obstacles(r,cl)
        background = np.zeros((501,801,3),np.uint8) 
        background.fill(255)
        frameSize = (800, 500)
        fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
        out = cv2.VideoWriter('Astar_visualisation.mp4',fourcc, 25, frameSize)
        #for every point that belongs within the obstacle
        for c in obstacles:
            x = c[0]*2
            y = c[1]*2
            background[(500-y,x)]=[0,0,255] #assigning a red colour for the obstacles
        out.write(background)
        if (check_obstacle(start,cl,r) or check_obstacle(goal,cl,r)):
            print('Enter the value again')
            continue
        else:
            correct = True
            visited_list, out, background, tc = A_star(node_queue , start, goal, obstacles_clearance, l, k, out, background, cl, r)
            path_list = []
            path_list.append(goal)
            goal_parent = visited_list[-1][3]
            child = goal_parent
            rechd_start = False
            while True:
                index = find_parent(child, visited_list)
                path_list.append(visited_list[index][-1])
                if(visited_list[index][2] == 1):
                    break
                child = visited_list[index][3]
            print('Path generated using back-tracking')
            for i in range(len(path_list)-1):
                x1 = path_list[i][0]*2
                y1 = 500 - path_list[i][1]*2
                x2 = path_list[i+1][0]*2
                y2 = 500 - path_list[i+1][1]*2
                cv2.line(background, (int(x1), int(y1)), (int(x2), int(y2)) , (0,0,0), 1 )
            for i in range(int(10*tc)):
                out.write(background)
            out.release()
            path = np.array(path_list)
            
            print('Visualisation video created')
            v = []
            o = np.array(obstacles)
            plt.xlim(0, 400)
            plt.ylim(0, 250)
            plt.scatter(o[:,0], o[:,1], c = 'r', s = 0.5)
            plt.plot(path[:,0], path[:,1], c = 'k')

            plt.savefig('Final_output.png')

if __name__ == '__main__':
    main()