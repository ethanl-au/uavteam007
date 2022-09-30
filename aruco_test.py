from .localise_subscriber import Localise
import cv2
import rospy
from image_node.msg import ROI
from geometry_msgs.msg import Point, PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class ArucoDetector():

    
    aruco_dict5 = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000)
    aruco_params = cv2.aruco.DetectorParameters_create()
    
    Aruco_info = [0,0,999] # [[x,y,ID],[...]] array of dimension 'N x 3'
    Published = False

    def __init__(self):
        self.aruco_pub = rospy.Publisher(
            '/processed_aruco/aruco_info', ROI, queue_size=10)

        self.br = CvBridge()

    ## Only calcs if it finds desired marker, might put into if loop to see if something is detected a number of times and then save it. 

    def find_aruco(self, frame, pose):

        desired_marker_ID = "7"

        (corners, ids, _) = cv2.aruco.detectMarkers(
            frame, self.aruco_dict5, parameters=self.aruco_params)
        aruco_globCoord = []
        ID = []
        localiser = Localise()
        if len(corners) > 0:
            ids = ids.flatten()

            for (marker_corner, marker_ID) in zip(corners, ids):
                corners = marker_corner.reshape((4, 2))
                (top_left, top_right, bottom_right, bottom_left) = corners

                top_right = (int(top_right[0]), int(top_right[1]))
                bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
                bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
                top_left = (int(top_left[0]), int(top_left[1]))

                cv2.line(frame, top_left, top_right, (0, 255, 0), 2)
                cv2.line(frame, top_right, bottom_right, (0, 255, 0), 2)
                cv2.line(frame, bottom_right, bottom_left, (0, 255, 0), 2)
                cv2.line(frame, bottom_left, top_left, (0, 255, 0), 2)

                # rospy.loginfo("Aruco detected, ID: {}".format(marker_ID))

                cv2.putText(frame, str(
                    marker_ID), (top_left[0], top_right[1] - 15), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 2)
                ## If marker ID is desired 
                if(str(marker_ID) == desired_marker_ID):
                    centroid = [((top_right[0]-top_left[0])/2)+top_left[0], ((bottom_left[1]-top_left[1])/2)+top_left[1]] #Image frame centroid
                    
                    global_coord = localiser.localisation(centroid,pose)
                    aruco_globCoord = global_coord
                    # rospy.loginfo("Aruco Global (x,y): {}".format([aruco_globCoord[0],aruco_globCoord[1]]))
                    ID = marker_ID #### Recording Marker ID
                               
                    self.Aruco_info = [aruco_globCoord[0],aruco_globCoord[1],ID] 
        return frame 
