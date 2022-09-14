#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, PoseStamped

def Roi_point_publisher():

    rospy.init_node('roi_node', anonymous = True)
    roi_publisher = rospy.Publisher('roi', PoseStamped, queue_size=1)
    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():    
        roi_waypoint = PoseStamped()
        roi_waypoint.pose.position.x = 0.0
        roi_waypoint.pose.position.y = 0.0
        roi_waypoint.pose.position.z = 2.0
        
        roi_waypoint.pose.orientation.x = 0.0
        roi_waypoint.pose.orientation.y = 0.0
        roi_waypoint.pose.orientation.z = 2.0
        roi_waypoint.pose.orientation.w = 0.0
        

        roi_publisher.publish(roi_waypoint)
        print("this is working")
        rate.sleep()
    
if __name__ == '__main__':
    try:
        Roi_point_publisher()
    except rospy.ROSInterruptionException:
        pass
