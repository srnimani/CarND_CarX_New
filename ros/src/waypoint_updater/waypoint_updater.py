#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from scipy.spatial import KDTree

import math
import numpy as np
import copy


'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MAX_DECEL = 5 # in m/ sec ^2


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # DONE: Add other member variables you need below
        self.pose = None
        self.base_lane = None
        self.stopline_wp_idx = -1
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.wp_distance_vector = [] # stores the distances between waypoints with respect
                                     # to the last waypoints
        
        # Subscribe to the needed messages
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # Done: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.loop()

    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.base_lane:
                self.publish_waypoints()
            rate.sleep()


    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y

        if self.waypoint_tree == None:
            return 0

        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        # Check if closest is ahead or behind the current position
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        # Equation for hyperplane through closesy coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx


    def publish_waypoints(self):
        if self.base_lane != None:
            final_lane = self.generate_lane()
            self.final_waypoints_pub.publish(final_lane)


    def generate_lane(self):
        lane = Lane()
        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_lane.waypoints[closest_idx:farthest_idx]
        #rospy.loginfo("Stopline_wp_idx %s", self.stopline_wp_idx)

        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
            lane.waypoints = base_waypoints
        else:
            #lane.waypoints = base_waypoints
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)

        return lane



    def O_decelerate_waypoints(self, waypoints, closest_idx):
        temp = []

        stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0) # Two waypoints back from stopline

        # Caculate all incremental distances between waypoints upto stop index
        self.wp_distance_vector = self.wp_distances(waypoints, stop_idx)

        # caluclate the total distance from the current pose till stop_idx    
        distance_to_stop = sum(self.wp_distance_vector)  
        rospy.loginfo("distance_to_stop  %s", distance_to_stop)

        # Get the current velocity of the car
        
        current_velocity = waypoints[0].twist.twist.linear.x
        decel_needed =  5

        rospy.loginfo("Current_vel  %s", current_velocity)


        # Calculuate the deceleration needed to stop the car
        #decel_needed = min((current_velocity ** 2 / (2 * distance_to_stop)), MAX_DECEL) # Remember v^2 = u^2 + 2as?

        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose
            # Decelerate till stop_idx is reached. Beyond that index set all velocities to 0
            if i < stop_idx:
                dist_to_next_wp = self.wp_distance_vector[i] # Need to calculate the speed adjustment between every adjacent waypoints
                new_vel2 = math.pow(current_velocity, 2) - 2 * decel_needed * dist_to_next_wp
                rospy.loginfo("distance_to_next wp  %s", dist_to_next_wp)
                if new_vel2 > 0:
                    vel = math.sqrt(new_vel2)
                else:
                    vel = 0
                current_velocity = vel
            else:
                vel = 0.

            #vel = 0.

            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            rospy.loginfo("vel  %s", p.twist.twist.linear.x)
            temp.append(p)

        return temp



    def decelerate_waypoints(self, waypoints, closest_idx):
        # make a copy of the waypoints.. need to adjust some velocities to 0
        decelerate_wps = copy.deepcopy(waypoints) 

        # Calculate the stop waypoint.. it's 2 points from the stopline to ensure that the car's nose is in
        stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0)

        # Caculate all incremental distances between waypoints upto stop index
        self.wp_distance_vector = self.wp_distances(decelerate_wps, stop_idx)

        # Calculuate the deceleration needed to stop the car
        #decel_needed = min((current_velocity ** 2 / (2 * distance_to_stop)), MAX_DECEL) # Remember v^2 = u^2 + 2as?

        for i in range(len(decelerate_wps)):
            # Decelerate till stop_idx is reached. Beyond that index set all velocities to 0
            if i < stop_idx:
                # caluclate the distance to stop from current waypoint till stop_idx    
                distance_to_stop = sum(self.wp_distance_vector[i:stop_idx])  
                rospy.loginfo("distance_to_stop  %s", distance_to_stop)
                vel = math.sqrt(2 * MAX_DECEL * distance_to_stop)     
                if vel < 1.0:
                    vel = 0.
            else:
                vel = 0.

            decelerate_wps[i].twist.twist.linear.x = vel
            rospy.loginfo("New velocity  %s", vel)

        return decelerate_wps



    def pose_cb(self, msg):
        # DONE: Implemented
        self.pose = msg 


    def waypoints_cb(self, waypoints):
        # DONE: Implemented
        self.base_lane = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] \
            for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)


    def traffic_cb(self, msg):
        # DONE: Callback for /traffic_waypoint message. Implemented
        self.stopline_wp_idx = msg.data 


    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x


    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity


    def wp_distances(self, waypoints, stop_idx):
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        distance_vector = []
        for i in range(stop_idx) :
            dist = dl(waypoints[i].pose.pose.position, waypoints[i+1].pose.pose.position)
            distance_vector.append(dist)
        return distance_vector



if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
