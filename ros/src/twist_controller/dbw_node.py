#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
from lowpass import LowPassFilter

from styx_msgs.msg import Lane

import math

from twist_controller import Controller

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

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # TODO: Create `TwistController` object
        # self.controller = TwistController(<Arguments you wish to provide>)

        parms = {
            'wheel_base'      : wheel_base,
            'steer_ratio'     : steer_ratio,
            'min_velocity'    : 0.,
            'max_lat_accel'   : max_lat_accel,
            'max_steer_angle' : max_steer_angle,
            'decel_limit'     : decel_limit,
            'accel_limit'     : accel_limit,
            'deadband'        : brake_deadband
        }

        self.controller = Controller(**parms)

        self.current_command  = None
        self.current_velocity = None
        self.dbw_enabled      = False

        # TODO: Subscribe to all the topics you need to

        rospy.Subscriber('/twist_cmd', TwistStamped, self.callback_twist_cmd)
        rospy.Subscriber('/current_velocity', TwistStamped, self.callback_current_velocity)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.callback_dbw_enabled)
        rospy.Subscriber('/final_waypoints', Lane, self.final_waypoints_cb)
        self.loop()


    def final_waypoints_cb(self, msg):  #should be OK
        self.final_waypoints = msg
        #rospy.logwarn(self.final_waypoints)
        #rospy.logwarn('dbw_node ' + str(self.final_waypoints.waypoints[1].twist.twist.linear.z))
        #rospy.logwarn(self.final_waypoints.waypoints[1].twist.twist.linear.x)

    def loop(self):
        rate = rospy.Rate(20) # 50Hz
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>)
            # if <dbw is enabled>:
            #   self.publish(throttle, brake, steer)
            if (self.current_command is not None) and (self.current_velocity is not None):
                #current velocity, target velocity, and target angle and pass into control
                #rospy.logwarn('Setting Velocity')
                linear_target  = self.current_command.twist.linear.x

                #rospy.logwarn(str(linear_target) )
                angular_target = self.current_command.twist.angular.z
                linear_current = self.current_velocity.twist.linear.x
                angular_current = self.current_velocity.twist.angular.z

                throttle, brake, steering = self.controller.control(linear_target, angular_target, linear_current)
                #rospy.logwarn(str(throttle))
                brake = brake * 500
                #rospy.logwarn(str(brake))
                
                
                # publish the control commands if dbw is enabled
                if self.dbw_enabled is True:

                    self.publish(throttle, brake, steering)
                else:
                    rospy.logwarn('resetting controller')
                    self.controller.reset()
            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)


    def callback_twist_cmd(self, msg):
        self.current_command = msg

    def callback_current_velocity(self, msg):
        self.current_velocity = msg

    def callback_dbw_enabled(self, msg):
        self.dbw_enabled = msg.data




if __name__ == '__main__':
    DBWNode()
