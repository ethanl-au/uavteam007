import rospy
from geometry_msgs.msg import Point, PoseStamped

roi_publisher = rospy.Publisher('roi', PoseStamped, queue_size=1)

roi_waypoint = PoseStamped()
roi_waypoint.pose.position.x = 0.0
roi_waypoint.pose.position.y = 0.0
roi_waypoint.pose.position.z = 0.0

roi_publisher.publish(roi_waypoint)
