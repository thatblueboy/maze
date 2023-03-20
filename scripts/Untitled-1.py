#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
import numpy as np
import math
import operator
import nav_msgs
from nav_msgs.msg import OccupancyGrid, MapMetaData, Path
from geometry_msgs.msg import PoseStamped
from shapely.geometry import Point, Polygon, LineString
from itertools import product,permutations
from tf.transformations import quaternion_from_euler
import time

# class Grid():
#     def __init__(self,grid) -> None:
#         super().__init__()
#         self.grid = grid
    
global grid
global value
global start_node, node, goal
global nodelist 
nodelist = []
value = 0
#start_node = []
node = []
#goal = []
# path_pub = rospy.Publisher('/Path', Path , queue_size=10)
grid = [
        [ 1, 0, 1, 1, 1, 1, 0, 1, 1, 1 ], 
       [ 1, 1, 1, 0, 1, 1, 1, 0, 1, 1 ], 
       [ 1, 1, 1, 0, 1, 1, 0, 1, 0, 1 ], 
       [ 0, 0, 1, 0, 1, 0, 0, 0, 0, 1 ], 
       [ 1, 1, 1, 0, 1, 1, 1, 0, 1, 0 ], 
       [ 1, 0, 1, 1, 1, 1, 0, 1, 0, 0 ], 
       [ 1, 0, 0, 0, 0, 1, 0, 0, 0, 1 ], 
       [ 1, 0, 1, 1, 1, 1, 0, 1, 1, 1 ], 
       [ 1, 1, 1, 0, 0, 0, 1, 0, 0, 1 ],
       [ 1, 1, 1, 0, 0, 0, 1, 0, 0, 1 ]
    ] 



# def map_callback(msg):
#     sub.unregister()
#     global grid
#     grid= np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
#     print(grid.shape)
#     x_arr=[]
#     y_arr=[]
#     x_arr_1=[]
#     y_arr_1=[]
#     nums = {}
#     a,b = grid.shape


#     print(nums)
#     for i  in range(a):
#         for j in range(b):
#             if(grid[i][j]==0):
#                 x_arr.append(i)
#                 y_arr.append(j)
#             if(grid[i][j]==16):
#                 x_arr_1.append(i)
#                 y_arr_1.append(j)
    
   
#     plt.scatter(x_arr_1, y_arr_1, color="blue")
#     plt.scatter(x_arr, y_arr, color="yellow")
#     plt.show()
#     #print(int(msg.info.height/8), int(msg.info.width/8),int(msg.info.height/2), int(msg.info.width/2))
#     run_astar(613,773,693,710)


# 0 --> Free
# 100 --> Obstacle
# -1 --> Unexplored

# for i in range(len(grid)):
#     for j in range(len(grid[0])):
#         if grid[i][j]==0:
#             plt.scatter(i,j,color='#000000')

class Node:
    def __init__(self,x,y):
        global value
        self.x=x
        self.y=y
        self.index=value
        value=value+1
        self.addNode(x,y)
    def addNode(self,x,y):
        global nodelist
        nodelist.append((x,y))


def run_astar(start_x,start_y,end_x,end_y):
    global start_node,node,goal
    global path_pub
    start_node=Node(start_x,start_y)
    goal=Point(end_x,end_y)
    node=[]
    final_path=astar()
    if not final_path:
        print("Not possible")
    else:
        final_path.reverse()
        pose = PoseStamped()
        pose_path=Path()
        for i in range(len(final_path)-1):
            pose.pose.position.x=final_path[i].x
            pose.pose.position.y=final_path[i].y
            pose.pose.position.z=0
            yaw=math.atan2(final_path[i+1].y-final_path[i].y, final_path[i+1].x-final_path[i].x)
            
            pose.pose.orientation=quaternion_from_euler(0, 0, yaw)
            pose.header.stamp= rospy.Time.now()
            pose_path.poses.append(pose)
        
        path_pub.publish(pose_path)
        
            
    print("End")
    

    # else:
    #     # print(final_path)
    #     # for k in range(len(final_path)-1):
    #     #     plt.plot(final_path[k].x, final_path[k].y, final_path[k+1].x, final_path[k+1].y)
    #     # plt.show()
        
    #     arr_x=[]
    #     arr_y=[]
    #     for j in final_path:
    #         arr_x.append(j.x)
    #         arr_y.append(j.y)
    #     plt.plot(arr_x, arr_y, color="red")
        
    #     plt.show()
       
        

        

def cost_distance(current_node):
    global start_node,node,goal
    dx=abs(current_node.x-start_node.x)
    dy=abs(current_node.y-start_node.y)
    h=(dx+dy)+(1.414-2)*min(dx, dy)
    return h

def distance(current_node): # calculating distance between current node and goal using diganol distance 
    global start_node,node,goal

    dx=abs(current_node.x-goal.x)
    dy=abs(current_node.y-goal.y)
    # h=(dx+dy)+(1.414-2)*min(dx, dy)
    h = math.sqrt(dx*dx +dy*dy)
    return h
global k
k = 0
# current[0]>=0 and current[0]<len(grid) and current[1]>=0 and current[1]<len(grid[0]) and grid[current[0]][current[1]]==1
def if_required(current):
    global start_node,node,goal
    global grid
    global value
    global nodelist
    
    #print(nodelist)
    #print(k)
    if (current  not in nodelist and current[0]>=0 and current[0]<len(grid) and current[1]>=0 and current[1]<len(grid[0]) and grid[current[0]][current[1]]==1):
        return True
    return False


def check_node(current):
    global start_node,node,goal
    global grid
    global value
    nearest_node={}
    if(if_required((int(current.x+1),int(current.y)))):
        nearest_node[Node(int(current.x+1),int(current.y)).index]=cost_distance(Point(current.x+1,current.y))
    if(if_required((int(current.x-1),int(current.y)))):
        nearest_node[Node(int(current.x-1),int(current.y)).index]=cost_distance(Point(current.x-1,current.y))
    if(if_required((int(current.x+1),int(current.y+1)))):
        nearest_node[Node(int(current.x+1),int(current.y+1)).index]=cost_distance(Point(current.x+1,current.y+1))
    if(if_required((int(current.x+1),int(current.y-1)))):
        nearest_node[Node(int(current.x+1),int(current.y-1)).index]=cost_distance(Point(current.x+1,current.y-1))
    if(if_required((int(current.x),int(current.y+1)))):
        nearest_node[Node(int(current.x),int(current.y+1)).index]=cost_distance(Point(current.x,current.y+1))
    if(if_required((int(current.x),int(current.y-1)))):
        nearest_node[Node(int(current.x+1),int(current.y-1)).index]=cost_distance(Point(current.x,current.y-1))
    if(if_required((int(current.x-1),int(current.y+1)))):
        nearest_node[Node(int(current.x-1),int(current.y+1)).index]=cost_distance(Point(current.x-1,current.y+1))
    if(if_required((int(current.x-1),int(current.y-1)))):
        nearest_node[Node(int(current.x-1),int(current.y-1)).index]=cost_distance(Point(current.x-1,current.y-1))

    return nearest_node


def astar():
    global start_node,node,goal
    global nodelist
    global grid
    global value
    open_list=[(0,0)]
    prev_node={}
    cost={start_node.index:0}
    f_score={start_node.index:distance(Point(start_node.x,start_node.y))}
    current_node=(start_node.x,start_node.y)
    while len(open_list)!=0:
        f_score=dict(sorted(f_score.items(), key=lambda x: x[1]))
        for l in f_score:
            if nodelist[l] in open_list:
                prev_node[l]=nodelist.index(current_node)
                current_node=nodelist[l]
                break
        # global k
        # k = k+1
        # #    print(grid)
        # if(k == 5):
        #     return False    
        if(current_node==(goal.x,goal.y)):
            return construct_path(prev_node,goal)
        open_list.remove(current_node)
        neighbour_node=check_node(Point(current_node[0],current_node[1]))
        for j in neighbour_node:

            current_cost=cost[nodelist.index(current_node)]
            if(current_cost<neighbour_node[j]):
                cost[j]=current_cost
                f_score[j]=current_cost+distance(Point(nodelist[j][0],nodelist[j][1]))
                if(nodelist[j] not in open_list):
                    open_list.append(nodelist[j])
                

    return False


def construct_path(prev_node,current_node):
    global start_node,node,goal
    global nodelist
    del prev_node[0]
    path=[Point(current_node.x,current_node.y)]
    current_node=(int(current_node.x),int(current_node.y))
    while nodelist.index(current_node) in prev_node.keys():
        current_node=nodelist[prev_node[nodelist.index(current_node)]]
        path.append(Point(current_node[0],current_node[1]))
    
    print(path)
    return path



# else:
#       plt.scatter(k.x,k.y,color='#ff0000')
#       for k in range(len(final_path)-1):
#         plt.plot(final_path[k].x, final_path[k].y, final_path[k+1].x, final_path[k+1].y)
# plt.show()


#ppx=[]
#ppy=[]
#for k in range(len(final_path)):
 #   ppx.append(final_path[k].x)
  #  ppy.append(final_path[k].y)
#plt.plot(ppx, ppy)
if __name__ == '__main__':
    # rospy.init_node('astar',anonymous=True)
    # sub = rospy.Subscriber("/map", OccupancyGrid, map_callback)
    
    # rospy.spin()
    run_astar(0,0,7,2)