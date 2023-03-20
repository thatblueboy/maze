#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int8
from std_msgs.msg import Int8MultiArray
from nav_msgs.msg import OccupancyGrid, MapMetaData, Path
import numpy as np
#figure out custom msgs


state = [0,0,0]
obs_info = {0:(740,650,760,620),1:(), 2:()} #(yt,xt, yb,xb)
count = 0
grid_msg = OccupancyGrid()

grid = []

def map_callback(msg):
    sub.unregister()
    global grid
    global grid_msg
    grid_msg.info = msg.info
    grid= np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
    unique = []
    a,b = grid.shape
    for i in range(a):
        for j in range(b):
            if grid[i][j] not in unique:
                unique.append(grid[i][j])
    obs_update(0)

    
def obs_update(msg):
    global map_pub
    global state
    global obs_info
    global grid
    global grid_msg
    index = msg #msg.data
    print(state[index])
    state[index] = int(not(state[index]))
    current_state = state[index]
    print(current_state)

    start_row,start_col,end_row,end_col = (740,500,760,620) #obs_info[index]
    for i in range(start_row,end_row+1):
        for j in range(start_col, end_col+1):
            grid[i][j] = current_state*16
        print(i)
    new_grid = []
    a,b = grid.shape
    for i in range(a):
        for j in range(b):
            new_grid.append(grid[i][j])
    grid_msg.data = new_grid
    map_pub.publish(grid_msg)


if __name__=='__main__':
    rospy.init_node("obstacle_manager")
    map_pub = rospy.Publisher('map2', OccupancyGrid, latch=True,queue_size=1)
    sub = rospy.Subscriber("/map", OccupancyGrid, map_callback)
    """while inp>=0:
        inp = int(input("Enter Value: "))
        if inp>=0:"""
    #obs_update(inp)
    #input_sub = rospy.Subscriber(keyboard_input, DataType, obs_update)
    rospy.spin()