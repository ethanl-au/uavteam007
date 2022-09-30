#!/usr/bin/env python3

import cv2

import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int8
from geometry_msgs.msg import Quaternion, PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import depthai as dai

import numpy as np
import math
from pathlib import Path
import sys
import cv2
import time
import argparse
import json
import blobconverter
from image_node.msg import ROI
from .aruco_test import ArucoDetector
from .localise_subscriber import Localise



class DepthaiCamera():
    res = [416, 416]
    fps = 20.0

    pub_topic_img = '/depthai_node/image/compressed'
    pub_topic_nn = '/depthai_node/NN_info'

    def __init__(self):
        self.pipeline = dai.Pipeline()
        self.pub_image = rospy.Publisher(
            self.pub_topic_img, CompressedImage, queue_size=10)

        self.pub_NN = rospy.Publisher(
            self.pub_topic_nn, ROI, queue_size=10) 

########### Subscribe to the opti-track position
        self.current_location = Quaternion()
        self.sub_pose = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.callback_pose)
################################
        rospy.loginfo("Publishing images to rostopic: {}".format(self.pub_topic_img))

        self.br = CvBridge()

        rospy.on_shutdown(lambda: self.shutdown())

    def rgb_camera(self):
        cam_rgb = self.pipeline.createColorCamera()
        cam_rgb.setPreviewSize(self.res[0], self.res[1])
        cam_rgb.setInterleaved(False)
        cam_rgb.setFps(self.fps)

        # Def xout / xin
        ctrl_in = self.pipeline.createXLinkIn()
        ctrl_in.setStreamName("cam_ctrl")
        ctrl_in.out.link(cam_rgb.inputControl)

        xout_rgb = self.pipeline.createXLinkOut()
        xout_rgb.setStreamName("video")

        cam_rgb.preview.link(xout_rgb.input)

##########################################################################################
######## Neural Network Config
        configPath = Path('/home/uavteam007/blob/yolov5.json')

        with configPath.open() as f:
            config = json.load(f)
        nnConfig = config.get("nn_config", {})

        if "input_size" in nnConfig:
            W, H = tuple(map(int, nnConfig.get("input_size").split('x')))

        # extract metadata
        metadata = nnConfig.get("NN_specific_metadata", {})
        classes = metadata.get("classes", {}) 
        coordinates = metadata.get("coordinates", {})
        anchors = metadata.get("anchors", {})
        anchorMasks = metadata.get("anchor_masks", {})
        iouThreshold = metadata.get("iou_threshold", {})
        confidenceThreshold = metadata.get("confidence_threshold", {})

        print(metadata)

###### Define Labels
        nnMappings = config.get("mappings", {})
        global labels
        labels = nnMappings.get("labels", {}) 
        print(labels)

        # get model path
        nnPath = '/home/uavteam007/blob/yolov5_dai.blob'
        # sync outputs
        syncNN = True

        detectionNetwork = self.pipeline.create(dai.node.YoloDetectionNetwork)
        nnOut = self.pipeline.create(dai.node.XLinkOut)
        nnOut.setStreamName("nn")

        detectionNetwork.setConfidenceThreshold(confidenceThreshold)
        detectionNetwork.setNumClasses(classes)
        detectionNetwork.setCoordinateSize(coordinates)
        detectionNetwork.setAnchors(anchors)
        detectionNetwork.setAnchorMasks(anchorMasks)
        detectionNetwork.setIouThreshold(iouThreshold)
        detectionNetwork.setBlobPath(nnPath)
        detectionNetwork.setNumInferenceThreads(2)
        detectionNetwork.input.setBlocking(False)

        # Linking 
        cam_rgb.preview.link(detectionNetwork.input)
        detectionNetwork.passthrough.link(xout_rgb.input)
        detectionNetwork.out.link(nnOut.input)
####################################################################################

    def run(self,aruco_detect):
        self.rgb_camera()
        

        frame = None
        detections = []
        startTime = time.monotonic()
        counter = 0
        color2 = (255, 255, 255)

        def frameNorm(frame, bbox):
            normVals = np.full(len(bbox), frame.shape[0])
            normVals[::2] = frame.shape[1]
            return (np.clip(np.array(bbox), 0, 1) * normVals).astype(int)


        def displayFrame(self, name, frame, detections, aruco_detect, current_pose):
            color = (255, 0, 0)
            target=0
            localiser = Localise()
            # print()
            if len(detections)==0:
                target_info=[0,0,0]
            else:
                target = detections[0].label # Figure out how to print multiple detected targets

            
            for detection in detections:
                bbox = frameNorm(frame, (detection.xmin, detection.ymin, detection.xmax, detection.ymax))
                cv2.putText(frame, labels[detection.label], (bbox[0] + 10, bbox[1] + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame, f"{int(detection.confidence * 100)}%", (bbox[0] + 10, bbox[1] + 40), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 2)
                
                                #Compute Location
##########################
                nn_centroid = [((detection.xmax-detection.xmin)/2)+detection.xmin , ((detection.ymax-detection.ymin)/2)+detection.ymin]
                

                nn_global_coord = localiser.localisation([nn_centroid[0]*416,nn_centroid[1]*416],current_pose)
                # rospy.loginfo("NN global coord: {}".format([nn_global_coord[0],nn_global_coord[1]]))
                
                ###### Neural Network Info ######
                nn_info_out = ROI() #[x,y,target]
                nn_info_out.target = detection.label #target
                nn_info_out.x = nn_global_coord[0]
                nn_info_out.y = nn_global_coord[1]

                self.pub_NN.publish(nn_info_out)
##########################
                
            # Show the frame
            self.publish_to_ros(frame,target,aruco_detect)

        with dai.Device(self.pipeline) as device:
            video = device.getOutputQueue(
                name="video", maxSize=1, blocking=False)
            qDet = device.getOutputQueue(
                name="nn", maxSize=4, blocking=False)


            while True:
                inDet = qDet.get()
                frame = video.get().getCvFrame()
                cv2.putText(frame, "NN fps: {:.2f}".format(counter / (time.monotonic() - startTime)),
                        (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color2)

                if inDet is not None:
                    detections = inDet.detections
                    counter += 1

                # SUBSCRIBE TO CURENT UAV POSE HERE??
##########################
                
                # current_pose = [self.current_location.x, self.current_location.y, self.current_location.z, self.current_location.w]
                current_pose = [1,-1.8,2,math.degrees(-0.999)] #Test
                #This Pose puts out zeros - pretty sure it's just because opti-track isn't publishing while testing.
##########################

                
                #Compute ArucoMarker And Draw on frame
##########################
                frame = aruco_detect.find_aruco(frame, current_pose)
##########################

                if frame is not None:
                    displayFrame(self,"rgb", frame, detections,aruco_detect,current_pose)
                #self.publish_to_ros(frame)
    
    # This function will check receive the current pose of the UAV constantly
    def callback_pose(self, msg_in):
		# Store the current position at all times so it can be accessed later
        self.current_location = msg_in.pose.orientation

    def publish_to_ros(self, frame, target,aruco_detect):
    ###### Compressed Image ######
        msg_out = CompressedImage()
        msg_out.header.stamp = rospy.Time.now()
        msg_out.format = "jpeg"
        msg_out.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()

    ###### Aruco Marker Info ######
        if str(aruco_detect.Aruco_info[2]) != str(999) and aruco_detect.Published == False: 
            aruco_info_out = ROI()
            aruco_info_out.target = aruco_detect.Aruco_info[2] # THIS ONLY PUBLISHES ONCE 
            aruco_info_out.x = aruco_detect.Aruco_info[0]    # INCORPORATE x number of detections 
            aruco_info_out.y = aruco_detect.Aruco_info[1]   # To publish message

            aruco_detect.aruco_pub.publish(aruco_info_out)
            aruco_detect.Published = True

        self.pub_image.publish(msg_out)
        # self.pub_NN.publish(nn_info_out)
        #Publish Aruco Info

    def shutdown(self):
        cv2.destroyAllWindows()


def main():
    rospy.init_node('depthai_node')

    dai_cam = DepthaiCamera()
    aruco_detect = ArucoDetector()

    while not rospy.is_shutdown():
        dai_cam.run(aruco_detect)

    dai_cam.shutdown()
