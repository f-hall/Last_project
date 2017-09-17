import rospy
from yaw_controller import YawController
from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.yawcontroller = YawController(2.8498, 14.8, 10.0, 3.0, 8.)
        self.pidcontroller = PID(0.125, 0.0001, 0.8)
        #pass

    def control(self, linear, angular, current, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        rospy.loginfo("CTR: control")
        rospy.loginfo("CTR: linear: %f", linear)
        rospy.loginfo("CTR: angular: %f", angular)
        rospy.loginfo("CTR: current: %f", current)

        '''
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
		min_speed = rospy.get_param('~min_speed', 10.0)
		'''

        # yawcontroller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        #angular = angular * 57.2
        steering_angle = self.yawcontroller.get_steering(linear, angular, current)
        rospy.loginfo("CTR: steering angle %f", steering_angle)

        # return throttle, brake, steer
        return 1.0, 0.0, ((steering_angle) * -1.0) # +0.03 in s_a

        #return 1., 0., 0.
