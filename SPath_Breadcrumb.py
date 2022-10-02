#!/usr/bin/env python3


from random import seed
import sys
from math import *
from turtle import distance
from argparse import ArgumentError

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus

from geometry_msgs.msg import Point, PoseStamped
from custom_msgs.msg import Imagery_Message
from nav_msgs.msg import Path
from std_msgs.msg import *

from spar_msgs.msg import FlightMotionAction, FlightMotionGoal
from breadcrumb.srv import RequestPath
from breadcrumb.srv import RequestPathRequest
from image_node.msg import ROI
from confirmation.msg import Confirmation



# This is getting a bit more complicated now, so we'll put our information in
# a class to keep track of all of our variables. This is not so much different
# to the previous methods, other than the fact that the class will operate
# within itself.
# i.e. it will have it's own publishers, subscribers, etc., that
# will call it's own functions as callbacks, etc.
class Guidance():
	def __init__(self, waypoints):
		#defines list that has cooridinates to ROI that have already been seen
		self.Seen_locations = []

		#set the constant for confirming with Imagery
		self.confirm_constant = 1

		#set tht Flag for when target is confirmed
		self.Confirmed = False
		self.confirmedTarget = 0
		#set the desired Aruco marker ID for landing
		#change this value on the day of flight
		#create variable in launch file
		#self.desired_marker_ID = 7

		#set the landing waypoint variable for when all targets dropped
		self.landing_waypoint = [0,0,0,0]

		#set flag when aruco landing is found
		self.safeLandingArucoFound = False

		#Create safe boundary default values incase not using launch file
		#default values and mapped varibles defined in launch file
		#default set to X < 4 or 3.5, Y < 2, Z <= 4
		self.inBoundsX = rospy.get_param("~inBoundsX", 3.5)
		self.inBoundsY = rospy.get_param("~inBoundsY", 3.0)
		self.inBoundsZ = rospy.get_param("~inBoundsZ", 4.0)
		
		# Make sure we have a valid waypoint list
		if not self.check_waypoints(waypoints):
			raise ArgumentError("Invalid waypoint list input!")

		# Internal counter to see what waypoint were are up to
		self.next_waypoint_index = 0

		# Set a flag to indicate that we are doing a specific inspection
		# and that we are not following our waypoint list
		# This will stop our "waypoint is reached" callback from firing
		# during the roi diversion and taking over our flight!
		self.performing_roi = False

		# Save the input waypoints
		self.waypoints = waypoints

		#Display the full path based on waypoints in Rviz
		self.display_path(waypoints, "/guidance/fullPath")

		# Make some space to record down our current location
		self.current_location = Point()
		self.current_yaw = FlightMotionGoal()
		# Set our linear and rotational velocities for the flight
		self.vel_linear = rospy.get_param("~vel_linear", 0.4)
		self.vel_yaw = rospy.get_param("~vel_yaw", 0.2)
		# Set our position and yaw waypoint accuracies
		self.accuracy_pos = rospy.get_param("~acc_pos", 0.3)
		self.accuracy_yaw = rospy.get_param("~acc_yaw", 0.1)
		#Set our search altitude default for easy change
		self.searchAltitude = rospy.get_param("~searchAlt", 2.0)

		# Create our action client
		action_ns = rospy.get_param("~action_topic", 'spar/flight')
		self.spar_client = actionlib.SimpleActionClient(action_ns, FlightMotionAction)
		rospy.loginfo("Waiting for spar...")
		self.spar_client.wait_for_server()

		#create breadcrumb service. Does not work if breadcrumb isnt working
		rospy.wait_for_service('/breadcrumb/request_path')
		self.srvc_bc = rospy.ServiceProxy('/breadcrumb/request_path', RequestPath)

		if not rospy.is_shutdown():
			# Good to go, start mission
			rospy.loginfo("Starting waypoint mission")

			# Setup first waypoint segment
			# XXX:	Another option would be to do "takeoff" and leave "waypoint_counter = 0" to
			#		begin the mission at the first waypoint after take-off
			rospy.loginfo("Initiating take-off")
			self.send_takeoff_motion(self.spar_client)

			self.send_wp(self.waypoints[0])
			self.next_waypoint_index += 1

			# Initialize and set list to store breadcrumb waypoints
			#set the flag for when the UAV is operting in breadcrumb mode
			self.breadcrumb_WPSnextIndex = 0
			self.breadcrumbMode = False
			self.breadcrumb_WPS = []

			# Setup a timer to check if our waypoint has completed at 20Hz
			self.timer = rospy.Timer( rospy.Duration(1.0/20.0), self.check_waypoint_status )
			# Callback to save "current location" such that we can perform and return
			# from a diversion to the correct location
			# XXX: These topics could be hard-coded to avoid using a launch file
			self.sub_pose = rospy.Subscriber("/uavasr/pose", PoseStamped, self.callback_pose)
			# Subscriber to catch "ROI" diversion commands
			# Subscriber for the actual ROI sent from imagery
			self.sub_imageryROI_targets = rospy.Subscriber('depthai_node/NN_info', ROI, self.callback_inspect_roi) 
			self.sub_imageryROI_aruco = rospy.Subscriber('processed_aruco/aruco_info', ROI, self.callback_setLanding_WP)
			#self.sub_roi = rospy.Subscriber("roi", PoseStamped, self.callback_inspect_roi)
			#self.sub_roi = rospy.Subscriber("roi", PoseStamped, self.callback_inspect_roi)
			#self.sup_roi_imagery = rospy.Subscriber("roi", Imagery_Message, self.callback_inspect_roi)

			#define the subscriber that listens for a confirmation from Imagery
			self.sub_confirmation_status = rospy.Subscriber('guidance', Confirmation, self.callback_set_confirm) 
			self.sub_yaw = rospy.Subscriber("/spar_msgs/FlightMotionGoal/yaw", FlightMotionGoal, self.callback_yaw)

			# XXX: Could have a publisher to output our waypoint progress
			# throughout the flight (should publish each time the waypoint
			# counter is increased). Note: will also need to import "Float32"
			# from "std_msgs.msg" in the header
			# self.pub_progress = rospy.Subscriber("~waypoint_progress", Float32, 10)

			# If shutdown is issued (eg. CTRL+C), cancel current
	 		# mission before rospy is shutdown.
			rospy.on_shutdown( lambda : self.shutdown() )

	# This function will check if a list of waypoints is in the format we expect
	def check_waypoints(self, wps):
		# Make sure waypoints are a list
		if not isinstance(wps, list):
			rospy.logwarn("Waypoints are not list")
			return False

		# Make sure we have at least one waypoint
		if len(wps) < 1:
			rospy.logwarn("Waypoints list is empty")
			return False

		# Check each of our waypoints
		for i in range(len(wps)):
			if not self.check_waypoint(wps[i]):
				rospy.logwarn("Waypoint %i did not pass check" % (i + 1))
				return False

		# If we haven't returned false yet, then waypoints look good!
		return True


	# This function will check if a waypoint is in the format we expect
	def check_waypoint(self, wp):
		# Make sure each waypoint is a list
		if not isinstance(wp, list):
			rospy.logwarn("Waypoint is not a list of coordinates")
			return False

		# Make sure each waypoint has 4 values
		if len(wp) != 4:
			rospy.logwarn("Waypoint has an invalid length (must be X/Y/Z/Yaw)")
			return False

		#check to make sure that waypoint provided is within the bounds of enviroment
		if abs(wp[0]) > self.inBoundsX or abs(wp[1]) > self.inBoundsY or wp[2] > self.inBoundsZ:
			rospy.logwarn("The Waypoint is out of environment Bounds")
			rospy.logwarn("Bounds should be -{}<X<{}/-{}<Y<{}/Z<{}".format(self.inBoundsX, self.inBoundsX, self.inBoundsY, self.inBoundsY, self.inBoundsZ))
			return False

		# If we haven't returned false yet, then waypoint looks valid!
		return True


	# This function will make sure we shut down the node as safely as possible
	def shutdown(self):
		# Unregister anything that needs it here
		self.sub_pose.unregister()
		self.sub_roi.unregister()
		self.sub_confirmation_status.unregister() 
		#self.sup_roi_imagery.unregister()
		self.sub_imageryROI_aruco.unregister()
		self.sub_imageryROI_targets.unregister()
		self.spar_client.cancel_goal()

		rospy.loginfo("Guidance stopped")


	# This function will check receive the current pose of the UAV constantly
	def callback_pose(self, msg_in):
		# Store the current position at all times so it can be accessed later
		self.current_location = msg_in.pose.position

	def callback_yaw(self, msg_in):
		#store the current yaw at all times so it can be used later
		self.current_yaw = msg_in.yaw	


	# This function will fire whenever a ROI pose message is sent
	# It is also responsible for handling the ROI "inspection task"
	def callback_inspect_roi(self, msg_in):

		# Calculate the distance from the new waypoint to each of the existing waypoints in the seen_location array
		distance_to_seen_locs = []
		for loc in self.Seen_locations:
			# x_diff = abs(msg_in.pose.position.x - loc[0])
			# y_diff = abs(msg_in.pose.position.y - loc[1])
			x_diff = abs(msg_in.x - loc[0])
			y_diff = abs(msg_in.y - loc[1])

			#Distance = pythagorian theorem (Hypotense = sqrt( a^2 + b^2))
			distance_to_seen_locs.append(sqrt(x_diff**2 + y_diff**2))

			#IF none of the distances are less that 10cm, run the code 

		#if not [msg_in.pose.position.x, msg_in.pose.position.y] in self.Seen_locations:
		if not [inside_boundary for inside_boundary in distance_to_seen_locs if inside_boundary < 0.10]: 
			
			# Set our flag that we are performing the diversion
			self.performing_roi = True

			
			rospy.loginfo("Starting diversion to ROI...")
			# Cancel the current goal (if there is one)
			self.spar_client.cancel_goal()
			# Record our current location so we can return to it later
			start_location = self.current_location
			# XXX:	It would also be a good idea to capture "current yaw" from
			#		the pose to maintain that throughout a diversion

			# Set the "diversion waypoint" (at yaw zero)
			#dwp = [msg_in.pose.position.x, msg_in.pose.position.y, self.searchAltitude, 0.0]
			dwp = [msg_in.x, msg_in.y, self.searchAltitude, 0.0]
			# Set the "return waypoint" (at yaw zero)
			rwp = [self.current_location.x, self.current_location.y, self.current_location.z, 0.0]

			# XXX: Could pause here for a moment with ( "rospy.sleep(...)" ) to make sure the UAV stops correctly
			rospy.sleep(rospy.Duration(5))

			self.send_wp(dwp)
			self.spar_client.wait_for_result()
			if self.spar_client.get_state() != GoalStatus.SUCCEEDED:
				# Something went wrong, cancel out of guidance!
				rospy.signal_shutdown("cancelled")
				return

			rospy.loginfo("Reached diversion ROI!")
			# XXX: Do something?
			#add waypoint to seen locations so that we dont divert at waypoint incase of multiple publishing
			self.Seen_locations.append([dwp[0], dwp[1]])
			rospy.sleep(rospy.Duration(5))

			#send signal flag to imagery to confirm the target that has been seen
			self.confirm_target(self.confirm_constant)
			if self.Confirmed == True:
				if self.confirmedTarget.target == 0 or self.confrimedTarget.target == 1:
					self.signal_to_payload(self.confirmedTarget.target)
				else:
					pass

			rospy.loginfo("Returning to flight plan...")
			
			self.send_wp(rwp)
			self.spar_client.wait_for_result()
			if self.spar_client.get_state() != GoalStatus.SUCCEEDED:
				# Something went wrong, cancel out of guidance!
				rospy.signal_shutdown("cancelled")
				return
			
			# "next_waypoint_index" represents the "next waypoint"
			# "next_waypoint_index - 1" represents the "current waypoint"
			rospy.loginfo("Resuming flight plan from waypoint %i!" % (self.next_waypoint_index - 1))
			self.send_wp(self.waypoints[self.next_waypoint_index - 1])
			# Unset our flag that we are performing a diversion
			# to allow the waypoint timer to take back over
			self.performing_roi = False
		else:
			#rospy.loginfo("coordinate already seen")
			pass

	# This function is for convinience to simply send out a new waypoint
	def send_wp(self, wp):
		# Make sure the waypoint is valid before continuing
		if not self.check_waypoint(wp):
			rospy.logwarn("Invalid waypoint, waypoint skipped...")
			return False
			#raise ArgumentError("Invalid waypoint input!")

		# Build the flight goal
		goal = FlightMotionGoal()
		goal.motion = FlightMotionGoal.MOTION_GOTO
		goal.position.x = wp[0]
		goal.position.y = wp[1]
		goal.position.z = wp[2]
		goal.yaw = wp[3]
		goal.velocity_vertical = self.vel_linear
		goal.velocity_horizontal = self.vel_linear
		goal.yawrate = self.vel_yaw
		goal.wait_for_convergence = True
		goal.position_radius = self.accuracy_pos
		goal.yaw_range = self.accuracy_yaw

		# For this function, we don't wait in the loop.
		# Instead we just send the waypoint and check up on it later
		# This checking is either with the "self.timer" for waypoints
		# or with direct calls during the ROI diversion
		self.spar_client.send_goal(goal)
		 # If shutdown is issued, cancel current mission before rospy is shutdown
		rospy.on_shutdown(lambda : self.spar_client.cancel_goal())


	# This function will fire whenever we recieve a timer event (te) from rospy.Timer()
	# The main purpose is to check if a waypoint has been reached,
	# and if so, send out the next waypoint to continue the mission
	def check_waypoint_status(self, te):
		current_yaw = self.current_yaw
		# If we're performing the ROI diversion, then don't do
		# anything here, as this is handled in that function
		if not self.performing_roi:
			# If the last segment has succeeded.
			# For more complex tasks, it might be necessary to also
			# check if you are in waypoint or diversion mode here.
			# Hint: really, we should check for other status states
			#		(such as aborted), as there are some states
			#		where we won't recover from, and should just exit
			if self.spar_client.get_state() == GoalStatus.SUCCEEDED:
				rospy.loginfo("Reached waypoint %i!" % (self.next_waypoint_index))

				# XXX:	Another check could go here to finish the mission early
				#		if "all" inspection tasks have been completed
				#		(Add in another "if" and make the waypoint counter check
				#		 an "elif" check instead.
				#		 i.e. if complete; elif more wps; else wps finished)
				if self.next_waypoint_index < (len(self.waypoints)):

					if not self.breadcrumbMode:
						#Set up a path request for breadcrumb
						req = RequestPathRequest()
						req.start.x = self.waypoints[self.next_waypoint_index-1][0] #Get the breadcrumb X
						req.start.y = self.waypoints[self.next_waypoint_index-1][1] #Get the breadcrumb Y
						req.start.z = self.waypoints[self.next_waypoint_index-1][2] #Get the breadcrumb Z
						req.end.x = self.waypoints[self.next_waypoint_index ][0]    #Get the next breadcrumb X in the list
						req.end.y = self.waypoints[self.next_waypoint_index ][1]    #Get the next breadcrumb Y in the list
						req.end.z = self.waypoints[self.next_waypoint_index ][2]    #Get the next breadcrumb Z in the list

						res = self.srvc_bc(req)

						# Breadcrumb will return a vector of poses if a solution was found
						# If no solution was found (i.e. no solution, or request bad
						# start/end), then breadcrumb returns and empty vector
						# XXX: You could also use res.path_sparse (see breadcrumb docs)
						breadcrumbWPS = []
						if len(res.path_sparse.poses) > 0:
							# Print the path to the screen
							rospy.loginfo("Segment {} to {}:".format(self.next_waypoint_index -1, self.next_waypoint_index))
							rospy.loginfo("[%0.2f;%0.2f;%0.2f] => [%0.2f;%0.2f;%0.2f]",
										req.start.x,req.start.y,req.start.z,
										req.end.x,req.end.y,req.end.z)

							# Loop through the solution returned from breadcrumb
							for i in range(len(res.path_sparse.poses)):
								rospy.loginfo("    [%0.2f;%0.2f;%0.2f]",
											res.path_sparse.poses[i].position.x,
											res.path_sparse.poses[i].position.y,
											res.path_sparse.poses[i].position.z)
								breadcrumbWPS.append([res.path_sparse.poses[i].position.x, res.path_sparse.poses[i].position.y, res.path_sparse.poses[i].position.z, 0.0])		

							#Display the Path
							# print(breadcrumbWPS)
							self.breadcrumb_WPS = breadcrumbWPS
							self.display_path(breadcrumbWPS, "/guidance/pathBreadcrumb")
							self.breadcrumbMode = True
							self.breadcrumb_WPSnextIndex = 0
							self.send_wp(self.breadcrumb_WPS[self.breadcrumb_WPSnextIndex])
							self.breadcrumb_WPSnextIndex +=1

						else:
							rospy.logerr("solution not found")						

					else:
						if self.breadcrumb_WPSnextIndex < (len(self.breadcrumb_WPS)):
							#There is a breadbrumb path, send waypoints to execute
							self.send_wp(self.breadcrumb_WPS[self.breadcrumb_WPSnextIndex])
							#Increment the waypoint counter
							self.breadcrumb_WPSnextIndex +=1
						else:
							#If finished with the breadcrumb mode waypoints increment the normal mode waypoints
							self.next_waypoint_index +=1
							self.breadcrumbMode = False	

				else:
					# Else the mission is over, shutdown and quit the node
					# XXX:	This could be used to restart the mission back to the
					#		first waypoint instead to restart the mission
					rospy.loginfo("Mission complete!")
					rospy.signal_shutdown("complete")
			elif (self.spar_client.get_state() == GoalStatus.PREEMPTED) or (self.spar_client.get_state() == GoalStatus.ABORTED) or (self.spar_client.get_state() == GoalStatus.REJECTED):
				rospy.loginfo("Mission cancelled!")
				rospy.signal_shutdown("cancelled")

	def display_path(self, wps, name):
		rospy.loginfo("Displaying path...")
		pub_path = rospy.Publisher(name, Path, queue_size=10, latch=True)
		msg = Path()
		msg.header.frame_id = "/map"
		msg.header.stamp = rospy.Time.now()

		for wp in wps:
			pose = PoseStamped()
			pose.pose.position.x = wp[0]
			pose.pose.position.y = wp[1]
			pose.pose.position.z = wp[2]

			pose.pose.orientation.w = 1.0
			pose.pose.orientation.x = 0.0
			pose.pose.orientation.y = 0.0
			pose.pose.orientation.z = 0.0

			msg.poses.append(pose)
		rospy.loginfo("Publishing path...")
		pub_path.publish(msg)	

	def send_landing_motion(self, spar_client):
		# Create our goal
		goal = FlightMotionGoal()
		goal.motion = FlightMotionGoal.MOTION_LAND
		goal.velocity_vertical = rospy.get_param("~speed", 0.2)		# Other velocity information is ignored
		# No other information is used

		# Send the goal
		rospy.loginfo("Sending goal motion...")
		spar_client.send_goal(goal)
		# If shutdown is issued, cancel current mission before rospy is shutdown
		rospy.on_shutdown(lambda : spar_client.cancel_goal())
		# Wait for the result of the goal
		spar_client.wait_for_result()

		# Output some feedback for our flight
		result = spar_client.get_state()
		if result == GoalStatus.SUCCEEDED:
			rospy.loginfo("Landing complete!")
		else:
			rospy.logerr("Landing failed!")

			# Detailed Feedback
			if result != GoalStatus.SUCCEEDED:
				if(result == GoalStatus.PENDING) or (result == GoalStatus.ACTIVE):
					rospy.loginfo("Sent command to cancel current mission")
				elif(result == GoalStatus.PREEMPTED):
					rospy.logwarn("The current mission was cancelled")
				elif(result == GoalStatus.ABORTED):
					rospy.logwarn("The current mission was aborted")
				elif(result == GoalStatus.RECALLED):
					rospy.logerr("Error: The current mission was recalled")
				elif(result == GoalStatus.REJECTED):
					rospy.logerr("Error: The current mission was rejected")
				else:
					rospy.logerr("Error: An unknown goal status was recieved")

	def send_takeoff_motion(self, spar_client):
		# Create our goal
		goal = FlightMotionGoal()
		goal.motion = FlightMotionGoal.MOTION_TAKEOFF
		goal.position.z = rospy.get_param("~height", 2.0)			# Other position information is ignored
		goal.velocity_vertical = rospy.get_param("~speed", 0.2)		# Other velocity information is ignored
		goal.wait_for_convergence = True							# Wait for our takeoff "waypoint" to be reached
		goal.position_radius = rospy.get_param("~position_radius", 0.1)
		goal.yaw_range = rospy.get_param("~yaw_range", 0.1)

		# Send the goal
		rospy.loginfo("Sending goal motion...")
		spar_client.send_goal(goal)
		# If shutdown is issued, cancel current mission before rospy is shutdown
		rospy.on_shutdown(lambda : spar_client.cancel_goal())
		# Wait for the result of the goal
		spar_client.wait_for_result()

		# Output some feedback for our flight
		result = spar_client.get_state()
		if result == GoalStatus.SUCCEEDED:
			rospy.loginfo("Take-off complete!")
		else:
			rospy.logerr("Take-off failed!")

			# Detailed Feedback
			if result != GoalStatus.SUCCEEDED:
				if(result == GoalStatus.PENDING) or (result == GoalStatus.ACTIVE):
					rospy.loginfo("Sent command to cancel current mission")
				elif(result == GoalStatus.PREEMPTED):
					rospy.logwarn("The current mission was cancelled")
				elif(result == GoalStatus.ABORTED):
					rospy.logwarn("The current mission was aborted")
				elif(result == GoalStatus.RECALLED):
					rospy.logerr("Error: The current mission was recalled")
				elif(result == GoalStatus.REJECTED):
					rospy.logerr("Error: The current mission was rejected")
				else:
					rospy.logerr("Error: An unknown goal status was recieved")	

	def signal_to_payload(self, msg_in):
		#create counter for payload when target is droped
		#create logic that if counter == 2 and safeLandingArucoFound == True unsubscribe/unregister from ROI node
		if msg_in == 0:
			rospy.loginfo("The bag has been found")
			drop_wp = [self.current_location.x, self.current_location.y, 0.5, 0.0]
			self.send_wp(drop_wp)
			#publish to payload the type of target found here
			self.publish_to_payload(msg_in)
			rospy.sleep(rospy.Duration(5))
			rospy.loginfo("Payload tracker has been dropped.")
			rospy.sleep(rospy.Duration(5))
		elif msg_in == 1:
			rospy.loginfo("The Person has been found")
			drop_wp = [self.current_location.x, self.current_location.y, 0.5, 0.0]
			self.send_wp(drop_wp)
			#publish to payload the type of target found here
			self.publish_to_payload(msg_in)
			rospy.sleep(rospy.Duration(5))
			rospy.loginfo("Payload epi-pen has been dropped.")
			rospy.sleep(rospy.Duration(5))	
						

	def confirm_target(self, input):
		#create a publisher that just sends message to confirm target to Imagery
		confirm_trg = rospy.Publisher('depthai_node', Int16, queue_size=10)
		msg_out = Int16()
		msg_out.data = input
		confirm_trg.publish(msg_out)

	def callback_set_confirm(self, msg_in):
		#update the confirmation flag to what is received from imagery
		self.Confirmed = msg_in.confirmed	
		self.confirmedTarget = msg_in.target

	def callback_setLanding_WP(self, msg_in):
		#set the coordinate componets of the aruco marker to that of the safe landing variable
		self.landing_waypoint[0] = msg_in.x
		self.landing_waypoint[1] = msg_in.y
		self.landing_waypoint[2] = self.searchAltitude
		self.landing_waypoint[3] = 0.0
		self.safeLandingArucoFound = True

	def publish_to_payload(self, input):
		#publish to payload the type of target found to drop correct payload
		target_found = rospy.Publisher('actuator_control/actuator_a', Int16, queue_size=10)
		msg_out = Int16()
		msg_out.data = input
		target_found.publish(msg_out)

def main(args):
	# Initialise ROS
	rospy.init_node('guidance')

	# List of waypoints
	# [X, Y, Z, Yaw]
	wps = [[-3.0,-2.5, 2.0, 0.0],
		   [-3.0, 2.5, 2.0, 1.5],
		   [-1.0, 2.5, 2.0, 0.0],
		   [-1.0,-2.5, 2.0, -1.5],
		   [ 1.0,-2.5, 2.0, 0.0],
		   [ 1.0, 2.5, 2.0, 1.5],
          [ 3.0, 2.5, 2.0, 0.0],
          [ 3.0, -2.5, 2.0, -1.5]]

# #		   [ 1.0, 2.0, 2.0, 1.5],
# 		   [-1.0, 2.0, 2.0, 0.0],
# 		   [-1.0,-1.0, 2.0, -1.5],
# 		   [ 1.8,-1.0, 2.0, 0.0],
# 		   [ 1.8, 1.0, 2.0, 1.5],
#            [ 0.0, 0.0, 2.0, 0.0]]
	# Create our guidance class option
	guide = Guidance(wps)
	#guide.send_landing_motion()
	# Spin!
	rospy.spin()


if __name__ == '__main__':
	try:
		main(sys.argv)
	except rospy.ROSInterruptException:
		pass

	print('')
