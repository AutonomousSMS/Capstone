#!/usr/bin/env python

import rospy
import os
import tf
import numpy as np
import time
import math
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

It will set a target velocity for those waypoints, using the traffic light waypoints as well.


'''

LOOKAHEAD_WPS                = 100    # Number of waypoints we will publish. You can change this number
VERBOSE                      = False  # Use rospy.logwarn in a couple of places
DEVELOPER                    = False  # Outputs time used by a subset of the algorithm
ARBITRARY_WAYPOINT_NUMBERING = False  # Allows for non-sequential numbering system
USE_TRAFFIC_LIGHTS           = True   # Uses Traffic light data
BRAKING_DISTANCE_START       = 30     # Intended to be the distance before the light at which braking starts 
BRAKING_DISTANCE_END         = 3      # Intended to be the distance before the light that the braking ends
VELOCITY_TARGET              = 11     # Velocity Target!

rospy.logwarn("Test")
class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        rospy.logwarn('Initializing Script')
        #Available rostopic list (unused subscriptions commented out)
        rospy.Subscriber('/base_waypoints',     Lane,           self.base_waypoints_cb)
        rospy.Subscriber('/current_pose',       PoseStamped,    self.current_pose_cb)  
        rospy.Subscriber('/current_velocity',   TwistStamped,   self.current_velocity_cb)
        #rospy.Subscriber('/image_color',   Lane,       '''needs self.something''')
        #rospy.Subscriber('/rosout'     Lane,       '''needs self.something''')
        #rospy.Subscriber('/rosout_agg,     Lane,       '''needs self.something''')
        #rospy.Subscriber('/tf          tfMessage,  '''needs self.something''')
        rospy.Subscriber('/traffic_waypoint',   Int32,          self.traffic_waypoint_cb)

        #rospy.Subscriber('/twist_cmd,      TwistStamped,   '''needs self.something''')
            
        #Likely not going to use the brake, steering, throttle command directly hereI
        
        
        #rospy.Subscriber('/vehicle/brake_cmd,      dbw_mkz_msgs/BrakeCmd,      '''needs self.something''')
        #rospy.Subscriber('/vehicle/steering_cmd,   dbw_mkz_msgs/SteeringCmd,   '''needs self.something''')
        #rospy.Subscriber('/vehicle/throttle_cmd,   dbw_mkz_msgs/ThrottleCmd,   '''needs self.something''')
        
        #Matthew - This is supposed to be the artificially known traffic lights I thinks?
        #rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_light_array_cb)
        
        #Matthew Younkins: I've commented out the line until I can determine when to use it
        #rospy.Subscriber('/obstacle_waypoint',     Lane,  self.obstacle_waypoint_cb)  #ok
        self.base_waypoints_cb
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.closest_waypoint_pub = rospy.Publisher('closest_waypoint', Int32, queue_size=1)  
        self.next_waypoint_pub = rospy.Publisher('next_waypoint', Int32, queue_size=1)  
        
        self.old_point = 0
        self.older_point = -1
        # Other member variables 
        rospy.spin()


    #def traffic_light_array_cb(self, msg):
        #self.traffic_light_array = TrafficLightArray
        #rospy.logwarn("Traffic Light Array " + str(TrafficLightArray))

    def current_pose_cb(self, msg):
        self.current_pose = msg.pose
        self.find_and_publish_final_waypoints()
        if VERBOSE:
            rospy.logwarn("Current Pose Called")
        
        
    def current_velocity_cb(self, msg):
        self.current_velocity = msg.twist.linear.x
        if VERBOSE:
            rospy.logwarn("Current Velocity Called")


    def base_waypoints_cb(self, Lane):  #should be OK
        self.base_waypoints = Lane.waypoints
        
    def traffic_waypoint_cb(self, msg):
        # These are the red lights 
        self.traffic_waypoint = msg.data
        rospy.logwarn(msg.data)
        if VERBOSE:
            rospy.logwarn("Traffic waypoint called, light message:" + str(self.traffic_waypoint))
        if self.traffic_waypoint >= 0:
            self.publish
            

    def obstacle_cb(self, msg):
        self.obstacle_waypoint = None
        
        
    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

        
    def find_and_publish_final_waypoints(self):
        waypoints = self.base_waypoints
        total_points = enumerate(waypoints)
        traffic_light_index = None
        #traffic_light_index = self.traffic_waypoint_cb

        #traffic_light_index = 350  #just an example here, need to remove this to get it to work
        if DEVELOPER:
            t1 = time.time()
       
        closest_waypoint_distance_sq = float("inf")
        closest_point = -1
        
        if traffic_light_index is not None:
            traffic_distance = math.sqrt((self.current_pose.position.x-waypoints[traffic_light_index].pose.pose.position.x)**2+
                                      (self.current_pose.position.y-waypoints[traffic_light_index].pose.pose.position.y)**2)
            #rospy.logwarn(traffic_distance)
        
        
        for index, waypoint in enumerate(self.base_waypoints):
            # no need to take the square root of here!  That reduced computation time from 18.6 to 18.4ms !!!!!!!!
            # I know you're impressed.
            waypoint_distance_sq = ((self.current_pose.position.x-waypoint.pose.pose.position.x)**2+
                                      (self.current_pose.position.y-waypoint.pose.pose.position.y)**2)
            #  waypoints[index].twist.twist.linear.z = index   #useful but not obvious, closest waypoint published on ros instead
            if (waypoint_distance_sq < closest_waypoint_distance_sq):
                closest_waypoint_distance_sq = waypoint_distance_sq
                closest_point = index
                
            if (traffic_light_index is not None) and (traffic_distance < BRAKING_DISTANCE_START):
                a = 1
                vel = self.current_velocity
                #rospy.logwarn('braking')
                #rospy.logwarn(current_velocity)
                #distance_between_waypoints = ???
                waypoints[index].twist.twist.linear.x = 0
                
                
                
                #if self.current_velocity < 0.5:
                #    waypoints[waypoint].twist.twist.linear.x = 0
                
            else:
                waypoints[index].twist.twist.linear.x = VELOCITY_TARGET
                  
        
        if DEVELOPER:
            t2 = time.time()
            rospy.logwarn("Time difference = " + str(t2-t1))

        if VERBOSE:
            rospy.logwarn("closest_point: " + str(closest_point)) 
        
        lane = Lane()
        #lane.header.frame_id = "header"
        lane.header.stamp = rospy.Time(0)
        lane.waypoints =  self.base_waypoints[closest_point:closest_point+LOOKAHEAD_WPS]
        self.final_waypoints_pub.publish(lane)
        self.closest_waypoint_pub.publish(closest_point)
        


        
if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
