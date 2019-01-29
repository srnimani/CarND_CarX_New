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
        mx = 0.8 # maximum throttle value
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        tau = 0.5 # 1/(2pi*tau) = cut-off frequency
        ts = 0.2 # sample_time
        self.vel_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = abs(decel_limit) # Lets use the absolute value to simplify feature equations
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()
        self.last_vel = 0


    def control(self, dbw_enabled, current_vel, target_vel, angular_vel):
        # DONE
        # Return throttle, brake, steer

        # If DBW not enabled (manual over-ride), return all 0's, driver takes control
        if not dbw_enabled:
        	self.throttle_controller.reset()
        	return 0., 0., 0.

        # Note the current and last time sample difference
        current_time = rospy.get_time()
        dt = current_time - self.last_time


        # Steering is controlled by angular velocity, current and target speeds
        steering = self.yaw_controller.get_steering(target_vel, angular_vel, current_vel)

        
        # Calculate the aceeleration / deceleration needed to achieve the desired speed
        current_vel = self.vel_lpf.filter(current_vel)

        vel_error = target_vel - current_vel

        #acceleration = (current_vel- self.last_vel) / dt # Current acceleration/ deceleration
        acceleration = (vel_error) / dt # Current acceleration/ deceleration
          
        # These two lines are for normal condition, will be later adjusted if we need to develerate or stop
        throttle = self.throttle_controller.step(vel_error, dt)
        brake = 0.0

        # Brake or decelerate only if the target velocity is lower than the current velocity
        # Brake value is in N/m and is calculated using car mass, acceleration and wheel radius
        # longitudinal force = mass of car * acceleration (or deceleration)
        # Torque = longitudinal force * wheel radius, which is supplied as brake value
        #
        # Further refinements can be done by adding the mass of fuel and the passengers to the mass
        # of the car in real world scenario
        #

        if target_vel == 0. and vel_error < 0.1: # This section is to hold the car @ Stop
            throttle = 0.
            brake = 700 # N*m, to hold Carla @ light.

        elif vel_error < 0 and current_vel != 0. : # Need to decelerate
            deceleration = min(abs(acceleration), self.decel_limit)
            brake = self.vehicle_mass * self.wheel_radius * deceleration * 20 # Does not still stop cleanly
            throttle = 0.0



        #Record the time and velocities for next cycle
        self.last_vel = current_vel
        self.last_time = current_time

        return throttle, brake, steering





        _