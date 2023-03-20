#!/usr/bin/env python3
""" Simple occupancy-grid-based mapping without localization.

Subscribed topics:
/scan

Published topics:
/map
/map_metadata

Author: Nathan Sprague
Version: 2/13/14
"""
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError


import numpy as np

class Map(object):
    """
    The Map class stores an occupancy grid as a two dimensional
    numpy array.

    Public instance variables:

        width      --  Number of columns in the occupancy grid.
        height     --  Number of rows in the occupancy grid.
        resolution --  Width of each grid square in meters.
        origin_x   --  Position of the grid cell (0,0) in
        origin_y   --    in the map coordinate system.
        grid       --  numpy array with height rows and width columns.


    Note that x increases with increasing column number and y increases
    with increasing row number.
    """

    def __init__(self, origin_x=-2.5, origin_y=-2.5, resolution=.1,
                 width=50, height=50):
        """ Construct an empty occupancy grid.

        Arguments: origin_x,
                   origin_y  -- The position of grid cell (0,0) in the
                                map coordinate frame.
                   resolution-- width and height of the grid cells
                                in meters.
                   width,
                   height    -- The grid will have height rows and width
                                columns cells.  width is the size of
                                the x-dimension and height is the size
                                of the y-dimension.

         The default arguments put (0,0) in the center of the grid.

        """
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.resolution = resolution
        self.width = width
        self.height = height
        self.grid = np.zeros((height, width))

    def to_message(self):
        """ Return a nav_msgs/OccupancyGrid representation of this map. """

        grid_msg = OccupancyGrid()

        # Set up the header.
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "map"

        # .info is a nav_msgs/MapMetaData message.
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.width
        grid_msg.info.height = self.height

        # Rotated maps are not supported... quaternion represents no
        # rotation.
        grid_msg.info.origin = Pose(Point(self.origin_x, self.origin_y, 0),
                               Quaternion(0, 0, 0, 1))

        # Flatten the numpy array into a list of integers from 0-100.
        # This assumes that the grid entries are probalities in the
        # range 0-1. This code will need to be modified if the grid
        # entries are given a different interpretation (like
        # log-odds).
        flat_grid = self.grid.reshape((self.grid.size,)) * 100
        grid_msg.data = list(np.round(flat_grid))
       
        return grid_msg

    def set_cell(self, x, y, val):
        """ Set the value of a cell in the grid.

        Arguments:
            x, y  - This is a point in the map coordinate frame.
            val   - This is the value that should be assigned to the
                    grid cell that contains (x,y).

        This would probably be a helpful method!  Feel free to throw out
        point that land outside of the grid.
        """
        pass



class Mapper(object):
    """
    The Mapper class creates a map from laser scan data.
    """

    def __init__(self):
        """ Start the mapper. """

        rospy.init_node('mapper')
        
        self._map = Map()
        self.bridge = CvBridge()
       

        # Setting the queue_size to 1 will prevent the subscriber from
        # buffering scan messages.  This is important because the
        # callback is likely to be too slow to keep up with the scan
        # messages. If we buffer those messages we will fall behind
        # and end up processing really old scans.  Better to just drop
        # old scans and always work with the most recent available.
        self.image_sub = rospy.Subscriber("cam/rgb/image_raw",Image,self.callback,queue_size=1)
        

        # Latched publishers are used for slow changing topics like
        # maps.  Data will sit on the topic until someone reads it.
        self._map_pub = rospy.Publisher('map', OccupancyGrid, latch=False,queue_size=1)
        self._map_data_pub = rospy.Publisher('map_metadata',
                                             MapMetaData, latch=False,queue_size=1)

        rospy.spin()


    def callback(self, data):
        """ Update the map on every scan callback. """
        try:
            image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            grey = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

            blur = cv2.GaussianBlur(grey,(35,35),0)
            



            ret,thresh= cv2.threshold(blur,127,255,cv2.THRESH_OTSU)
            
            
            

            cnts = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = cnts[0] if len(cnts) == 2 else cnts[1]
            cnts = sorted(cnts, key=cv2.contourArea, reverse=True)
            for c in cnts:
                x,y,w,h = cv2.boundingRect(c)
                thresh = thresh[y:y+h, x:x+w]
                break

            
            thresh[thresh == 255] = 100
            self._map.width = len(thresh[0])
            self._map.height = len(thresh)
            self._map.grid = thresh



            # Fill some cells in the map just so we can see that something is
            # being published.





            # Now that the map is updated, publish it!
            rospy.loginfo("Scan is processed, publishing updated map.")
            self.publish_map()
        except Exception as e:
            rospy.loginfo(e)
            pass


    def publish_map(self):
        """ Publish the map. """
        grid_msg = self._map.to_message()
        self._map_data_pub.publish(grid_msg.info)
        self._map_pub.publish(grid_msg)


if __name__ == '__main__':
    try:
        m = Mapper()
    except rospy.ROSInterruptException:
        rospy.loginfo("error")
        pass
