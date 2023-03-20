import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError


import numpy as np


class Localiser(object):
    """
    The Mapper class creates a map from laser scan data.
    """

    def __init__(self):
        """ Start the mapper. """

        rospy.init_node('loacaliser')
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary,self.parameters)
        self.cam_width=None
        self.cam_height=None
        self.map_width=None
        self.map_height=None
     
     
        

        self.bridge = CvBridge()
       

        # Setting the queue_size to 1 will prevent the subscriber from
        # buffering scan messages.  This is important because the
        # callback is likely to be too slow to keep up with the scan
        # messages. If we buffer those messages we will fall behind
        # and end up processing really old scans.  Better to just drop
        # old scans and always work with the most recent available.
        self.image_sub = rospy.Subscriber("cv_camera/image_raw",Image,self.callback,queue_size=1)
        self.map_sub = rospy.Subscriber("map",OccupancyGrid,self.map_callback,queue_size=1)
        

        # Latched publishers are used for slow changing topics like
        # maps.  Data will sit on the topic until someone reads it.
        
       

        rospy.spin()
    def getCord(self,markerCorners,markerIds):
            out ={}
            for i in range(len(markerIds) if len(markerIds) < 4 else 4):
                x = markerCorners[0][0][i][0]
                y = markerCorners[0][0][i][1]
                id = markerIds[i][0]
                out[id]=(x,y)
            if len(out) == 4 and self.map_height != None and self.map_width != None:
                x0= out[2][0]
                y0= out[2][1]
                x1= out[4][0]
                y1= out[4][1]
                self.cam_height=abs(out[2][1]-out[1][1])
                self.cam_width=abs(out[2][0]-out[3][0])
                ry = self.map_height/self.cam_height
                rx = self.map_width/self.cam_width
                return (rx*abs(x1-x0),ry*(y1-y0))
            return None


    def callback(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            markerCorners, markerIds, rejectedCandidates = self.detector.detectMarkers(gray)
            if markerIds is not None:
                cv2.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)
                rospy.loginfo(self.getCord(markerCorners,markerIds))
           
             
        except Exception as e:
            rospy.loginfo(e)
            pass
    def map_callback(self,data):
         self.map_width= data.info.width
         self.map_height= data.info.height
        
         
		
   
	
  
	
if __name__ == '__main__':
	try:
		Localiser()
	except rospy.ROSInterruptException:
		pass