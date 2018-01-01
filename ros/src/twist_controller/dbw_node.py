#!/usr/bin/env python
from twist_controller import TwistController
from std_msgs.msg import Bool
#The accelerator request is called 'Throttle', Oops!  Have to mix here...
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
from styx_msgs.msg import Lane
from styx_msgs.msg import Lane
import math
import rospy



'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''



class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        accel_limit = rospy.get_param('~accel_limit', 1.)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)
        min_speed = 0.01
        
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        
        # NOTE - change vehicle mass with additional passengers and fuel.
        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        self.vehicle_mass= rospy.get_param('~vehicle_mass', 1736.35)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        
        
        self.TwistController = TwistController(wheel_base,steer_ratio,min_speed,max_lat_accel,
                            max_steer_angle, decel_limit, accel_limit, brake_deadband)

                            
        # Subscribe to all the topics you need to
        
        self.dbw_enabled = None   #should probably be false in real life as default
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb, queue_size=1)
        
        #rospy.Subscriber('/base_waypoints',     Lane,           self.base_waypoints_cb)
        #rospy.Subscriber('/current_pose',       PoseStamped,    self.current_pose_cb)  
        
        self.current_velocity = None
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb, queue_size=1)
        
        #rospy.Subscriber('/image_color',   Lane,       '''needs self.something''')
        #rospy.Subscriber('/final_waypoints',   Lane,       self.final_waypoints_pub)
        #rospy.Subscriber('/rosout'     Lane,       '''needs self.something''')
        #rospy.Subscriber('/rosout_agg,     Lane,       '''needs self.something''')
        #rospy.Subscriber('/tf          tfMessage,  '''needs self.something''')
        #rospy.Subscriber('/traffic_waypoint',   Int32,      self.traffic_waypoint_cb)

        self.twist_cmd = None
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb, queue_size=1)
       
        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
        self.accel_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)
        
        
        #rospy.Subscriber('/vehicle/traffic_lights, TrafficLightArray,'''needs self.something''')
        #rospy.Subscriber('/obstacle_waypoint',     Lane,  self.obstacle_waypoint_cb)  #ok
        self.reset = False
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)  
        self.last_time = rospy.get_time()   #initialize for loop below
        self.loop()

        
    def dbw_enabled_cb(self, dbw_enabled):
        self.dbw_enabled = dbw_enabled

    def twist_cmd_cb(self, twist_cmd):
        self.twist_cmd = twist_cmd

    def current_velocity_cb(self, current_velocity):
        self.current_velocity = current_velocity
        
        
    def loop(self):
        rate = rospy.Rate(25) 
        while not rospy.is_shutdown():
        
        
            time = rospy.get_time()
            dT = time - self.last_time
            self.last_time = time
               
            accel_pedal, brake, steering = self.TwistController.control(
                twist_cmd=self.twist_cmd, current_velocity=self.current_velocity,
                del_time=dT, vehicle_mass=self.vehicle_mass)
            
            if self.reset:
                rospy.logwarn('Resetting Controllers in dbw_node')
                self.TwistController.reset()
                self.reset_flag = False
                
            elif self.dbw_enabled:
                rospy.logwarn('publishing pedal and steering')
                self.publish(accel_pedal, brake, steering)
        
        rate.sleep()

    def publish(self, accel_pedal, brake, steer):
        acmd = ThrottleCmd()
        acmd.enable = True
        acmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        acmd.pedal_cmd = accel_pedal
        rospy.logwarn(acmd)
        self.throttle_pub.publish(acmd)
        
        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        rospy.logwarn(bcmd)
        self.brake_pub.publish(bcmd)
        
        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        rospy.logwarn(scmd)
        self.steer_pub.publish(scmd)

        


if __name__ == '__main__':
    DBWNode()