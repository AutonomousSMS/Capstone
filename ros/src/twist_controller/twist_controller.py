import rospy
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController


GAS_DENSITY = 2.858
ONE_MPH = 0.44704



class TwistController(object):
    def __init__(self, *args, **kwargs):
    
        self.wheel_base         = args[0]
        rospy.logwarn('wheel_base=' + str(self.wheel_base))
        self.steer_ratio        = args[1]
        rospy.logwarn('steer_ratio=' + str(self.steer_ratio))
        self.min_speed          = args[2]
        rospy.logwarn('min_speed=' + str(self.min_speed))
        self.max_lat_accel      = args[3]
        rospy.logwarn('max_lat_accel=' + str(self.max_lat_accel))
        self.max_steer_angle    = args[4]
        rospy.logwarn('max_steer_angle=' + str(self.max_steer_angle))
        self.decel_limit        = args[5]
        rospy.logwarn('decel_limit=' + str(self.decel_limit))
        self.accel_limit        = args[6]
        rospy.logwarn('accel_limit=' + str(self.accel_limit))
        
        self.yaw_controller = YawController(wheel_base=self.wheel_base,
            steer_ratio=self.steer_ratio, min_speed=self.min_speed,
            max_lat_accel=self.max_lat_accel, max_steer_angle=self.max_steer_angle)
            
         
        self.pid = PID(kp = 1, ki = 0, kd = 0, mn = self.decel_limit,   mx = self.accel_limit)
        
        self.brake_deadband      = args[7]
        rospy.logwarn('brake_deadband=' + str(self.brake_deadband))
        
        
    def reset(self):
        self.pid.reset()

        
    def control(self, twist_cmd, current_velocity, del_time, vehicle_mass):
        lin_vel = twist_cmd.twist.linear.x
        ang_vel = twist_cmd.twist.angular.z
        vel_err = current_velocity.twist.linear.x

        steering_angle = self.yaw_controller.get_steering(abs(twist_cmd.twist.linear.x),
                            twist_cmd.twist.angular.z, current_velocity.twist.linear.x)
        
        accelerationValue = 0.1  #need to put in PID for real
        
        if acceleration > 0.0:   #probably want to make this a slightly positive number
            accel_pedal = acceleration / vehicle_mass  #needs a scaling function
            brake_pedal = 0.0
            
        else:
            accel_pedal = 0.0
            brake_pedal = -acceleration / vehicle_mass  #needs a scaling function

        
        return accel_pedal, brake_pedal, steering_angle
