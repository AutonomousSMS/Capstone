#!/usr/bin/env python

import rospy
import os
import tf
import numpy as np
import time
import math
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
		
	#Available rostopic list (unused subscriptions commented out)
	rospy.Subscriber('/base_waypoints', 	Lane, 		self.base_waypoints_cb)
	rospy.Subscriber('/current_pose',   	PoseStamped,	self.current_pose_cb)  
	rospy.Subscriber('/current_velocity', 	TwistStamped,	self.current_velocity_cb)
	#rospy.Subscriber('/image_color', 	Lane,		'''needs self.something''')
	#rospy.Subscriber('/rosout'		Lane,		'''needs self.something''')
	#rospy.Subscriber('/rosout_agg, 	Lane,		'''needs self.something''')
	#rospy.Subscriber('/tf			tfMessage,	'''needs self.something''')
	rospy.Subscriber('/traffic_waypoint', 	Int32, 		self.traffic_waypoint_cb)
	#rospy.Subscriber('/twist_cmd, 		TwistStamped,	'''needs self.something''')
		
	#I'm not sure yet how to get the vehicle subscriber working ...
	#rospy.Subscriber('/vehicle/brake_cmd,		dbw_mkz_msgs/BrakeCmd,		'''needs self.something''')
	#rospy.Subscriber('/vehicle/steering_cmd,	dbw_mkz_msgs/SteeringCmd,	'''needs self.something''')
	#rospy.Subscriber('/vehicle/throttle_cmd,	dbw_mkz_msgs/ThrottleCmd,	'''needs self.something''')
	#rospy.Subscriber('/vehicle/traffic_lights,	TrafficLightArray,'''needs self.something''')
	
	#Matthew Younkins: I've commented out the line below because
	#I don't know where this is coming from yet
	#rospy.Subscriber('/obstacle_waypoint', 	Int32,	self.obstacle_waypoint_cb)  #ok
	
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)  

        # Other member variables 

        rospy.spin()

    def current_pose_cb(self, msg):
        self.current_pose = msg.pose
        
    def current_velocity_cb(self, msg):
        self.current_velocity = msg.twist.linear.x

    def base_waypoints_cb(self, msg):
        self.base_waypoint = msg.data
        
    def traffic_waypoint_cb(self, msg):   #I believe this is actually the red lights
        self.traffic_waypoint = msg.data

    def obstacle_cb(self, msg):
        self.obstacle_waypoint = None
        
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
