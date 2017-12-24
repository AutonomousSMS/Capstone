#!/usr/bin/env python

import rospy
import os
import tf
import numpy as np
import time
import math
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint




'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
DECEL_RATE = 3.0	# Deceleration rate.



class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
		
		'''  From the rostopic list, these are what are available:
		/base_waypoints, 		styx_msgs/Lane				Used
		/current_pose			geometry_msgs/PoseStamped	Used
		/current_velocity  		geometry_msgs/TwistStamped  Used
		/final_waypoints   		styx_msgs/Lane
		/image_color			styx_msgs/Lane
		/rosout					styx_msgs/Lane
		/rosout_agg				styx_msgs/Lane
		/tf						tf/tfMessage
		/traffic_waypoint		styx_msgs/Lane				Used
		/twist_cmd				geometry_msgs/TwistStamped
		/vehicle/brake_cmd		dbw_mkz_msgs/BrakeCmd
		/vehicle/steering_cmd	dbw_mkz_msgs/SteeringCmd
		/vehicle/throttle_cmd	dbw_mkz_msgs/ThrottleCmd
		/vehicle/traffic_lights	styx_msgs/TrafficLightArray
		'''
		
		# Subscribers in rospy
		rospy.Subscriber('/base_waypoints', 	Lane, 			self.waypoints_cb)
		rospy.Subscriber('/current_pose',   	PoseStamped,	self.pose_cb)  
		rospy.Subscriber('/current_velocity', 	TwistStamped,	self.current_velocity_cb)
        

        # Subscriber for /traffic_waypoint and /obstacle_waypoint
		rospy.Subscriber('/traffic_waypoint', 	Int32, 	self.traffic_cb)
		
		#Matthew Younkins: I've commented out the line below because
		#I don't know where this waypoint is coming from yet
		#rospy.Subscriber('/obstacle_waypoint', 	Int32,	self.obstacle_cb)  #ok
		
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)  

        # Other member variables 

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
