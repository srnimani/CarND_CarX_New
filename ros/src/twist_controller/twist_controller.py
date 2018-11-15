from pid import PID 
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, \
    accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # DONE: Implemented
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)

        kp = 0.3
        ki = 0.1
        kd = 0.
        mn = 0. # minimum throttle value
        mx = 0.4 # maximum throttle value
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        tau = 0.5 # 1/(2pi*tau) = cut-off frequency
        ts = 0.2 # sample_time
        self.vel_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()
        self.last_vel = 0


    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        # DONE
        # Return throttle, brake, steer

        if not dbw_enabled:
        	self.throttle_controller.reset()
        	return 0., 0., 0.

        current_vel = self.vel_lpf.filter(current_vel)

        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        vel_error = linear_vel - current_vel

        current_time = rospy.get_time()
        dt = current_time - self.last_time
        acceleration = (current_vel- self.last_vel) / dt

        self.last_vel = current_vel
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, dt)
        brake = 0



        """

        if linear_vel == 0. and vel_error < 0.1:
        	throttle = 0.
        	brake = 700 # N*m, to hold Carla @ light.


        elif linear_vel < 0.1 and vel_error < 0.:
        	throttle = 0.
        	decel = max(vel_error, self.decel_limit)
        	brake = abs(decel)*self.vehicle_mass*self.wheel_radius # Torque N*m

        """
        # The following code is if the car needs to slowdown or stop

        if vel_error < 0.: # Need to decelerate
            #rospy.loginfo("Error %s", vel_error)
            decel = abs(min(abs(acceleration), abs(self.decel_limit)))
            brake = decel * self.vehicle_mass * self.wheel_radius
            #brake = 100 * self.vehicle_mass * self.wheel_radius * 10
            #rospy.loginfo("Brake %s", brake)
            throttle = 0.
        elif linear_vel == 0. and vel_error < 0.1:
        	brake = 700 # Nm, to hold Carla @ light.
        	throttle = 0.

        #rospy.loginfo("Brake %s", brake)

        return throttle, brake, steering





        _