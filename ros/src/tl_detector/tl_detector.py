#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, Header
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import numpy as np
import math

STATE_COUNT_THRESHOLD = 3

TOO_VERBOSE = False
VERBOSE = True


class TLDetector(object):

    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        
        #rospy.logwarn(sub3)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)
        sub7 = rospy.Subscriber('/final_waypoints', Lane, self.final_waypoints_cb)
        sub8 = rospy.Subscriber('/closest_waypoint' ,Int32, self.closest_waypoint_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def final_waypoints_cb(self, msg):  #should be OK
        self.final_waypoints = msg

        if VERBOSE:
            #rospy.logwarn(self.final_waypoints)
            #rospy.logwarn('tl_detect ' + str(self.final_waypoints.waypoints[1].twist.twist.linear.z))
            #rospy.logwarn(self.final_waypoints.waypoints[1].twist.twist.linear.x)

            #Pose is the location.  1 is the closest waypoint published by waypoint_updater
            xLoc = self.final_waypoints.waypoints[1].pose.pose.position.x
            yLoc = self.final_waypoints.waypoints[1].pose.pose.position.y
            zLoc = self.final_waypoints.waypoints[1].pose.pose.position.z
            rospy.logwarn('Pose:  ' + str(xLoc) + ' ' + str(yLoc)+ ' ' + str(zLoc))

            #Twist is desired velocity
            xLoc = self.final_waypoints.waypoints[1].twist.twist.linear.x
            yLoc = self.final_waypoints.waypoints[1].twist.twist.linear.y
            zLoc = self.final_waypoints.waypoints[1].twist.twist.linear.z
            rospy.logwarn('Twist: ' + str(xLoc) + ' ' + str(yLoc)+ ' ' + str(zLoc))


    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

        

    def closest_waypoint_cb(self, msg):
        self.closest_waypoint = msg
        



    def traffic_cb(self, msg):
        self.lights = msg.lights
        closest_light_distance_sq = float("inf")
        closest_light = -1
        for index, light in enumerate(self.lights):

            if TOO_VERBOSE:
                #rospy.logwarn(self.lights[index].pose.pose.position.x)
                xLoc = self.lights[index].pose.pose.position.x
                yLoc = self.lights[index].pose.pose.position.y
                rospy.logwarn(str(index) + ': ' + str(xLoc) + ' ' + str(yLoc))


            light_distance_sq = ((self.final_waypoints.waypoints[1].pose.pose.position.x-
                                        self.lights[index].pose.pose.position.x)**2+
                                (self.final_waypoints.waypoints[1].pose.pose.position.y-
                                        self.lights[index].pose.pose.position.y)**2)

        
            if (light_distance_sq < closest_light_distance_sq):
                closest_light_distance_sq = light_distance_sq
                closest_light = index
        if VERBOSE:
            rospy.logwarn('Closest Light Index: ' + str(closest_light))
        #        
        #    if (traffic_light_index is not None) and (traffic_distance < BRAKING_DISTANCE_START):
        #        a = 1
        #        vel = self.current_velocity
        #        #rospy.logwarn('braking')
        #        #rospy.logwarn(current_velocity)
        #        #distance_between_waypoints = ???
        #        waypoints[index].twist.twist.linear.x = 0
                

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
            rospy.logwarn('Red Light Detected A:' + str(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
            rospy.logwarn('Red Light Detected B:' + str(light_wp))
        self.state_count += 1

   
    def distance(self,x1, y1, x2, y2):
        return math.sqrt((x2-x1)**2 + (y2-y1)**2)


    def get_next_waypoint(self, pose, waypoints):
        """this gives the closest waypoint in the car's heading direction"""
        rospy.logwarn(self.closest_waypoint.data)
        wp_x = waypoints[self.closest_waypoint.data].pose.pose.position.x
        wp_y = waypoints[self.closest_waypoint.data].pose.pose.position.y
        heading_dir = math.atan2((wp_y - pose.position.y), (wp_x - pose.position.x ))
        angle = abs(pose.position.z - heading_dir)
        if angle > math.pi / 4.0:
            closest_wp += 1

        return closest_wp


    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location
        Args:
            point_in_world (Point): 3D location of a point in the world
        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image
        """
        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        # get transform between pose of camera and world frame
        trans = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                  "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link",
                  "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        

        x = 0
        y = 0

        return (x, y)


    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return TrafficLight.RED

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        if (self.listener is not None):
            x, y = self.project_to_image_plane(light.pose.pose.position)

        #Get classification
        return self.light_classifier.get_classification(cv_image)


    def create_pose (self, x, y, z, yaw = 0.0):
        pose = PoseStamped()

        pose.header = Header()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'world'

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        q = tf.transformations.quaternion_from_euler(0.0, 0.0, math.pi * yaw / 180.0)
        pose.pose.orientation = Quaternion(*q)

        return pose


    def create_light(self, x, y, z, yaw, state):
        light = TrafficLight()

        light.header = Header()
        light.header.stamp = rospy.Time.now()
        light.header.frame_id = 'world'

        light.pose = self.create_pose(x,y,z, yaw)
        light.state = state

        return light 


    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        visibility_dist_max = 80.0
        min_dist = float('inf')
        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']

        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose, self.waypoints.waypoints)
            # (car_position)
        #TODO find the closest visible traffic light (if one exists)

        for stop_light_pos in stop_line_positions:
            light_tmp = self.create_light(stop_light_pos[0], stop_light_pos[1], 0.0, 0.0, TrafficLight.UNKNOWN)
            light_pos = self.get_closest_waypoint(light_tmp.pose.pose, self.waypoints.waypoints)
            dist = self.distance(self.waypoints.waypoints[car_position].pose.pose.position.x,
                                 self.waypoints.waypoints[car_position].pose.pose.position.y,
                                 self.waypoints.waypoints[light_pos].pose.pose.position.x,
                                 self.waypoints.waypoints[light_pos].pose.pose.position.y)

            if dist < min_dist and dist < visibility_dist_max and car_position < light_pos:
                min_dist = dist
                light = light_tmp
                light_wp = light_pos



        if light:
            state = self.get_light_state(light)
            return light_wp, state
        
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
