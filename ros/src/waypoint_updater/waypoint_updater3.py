#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import tf

import math

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


class WaypointUpdater(object):
    def __init__(self):
        self.wp = []
        self.count_waypoint = 0
        self.waypoints = 0
        self.pub_wp = Lane()
        self.current_pose = np.array([0.0,0.0,0.0])
        self.current_orientation = np.array([0.0, 0.0, 0.0, 0.0])
        self.current_dir = np.array([0.0,0.0,0.0,0.0])
        self.numpy_list = np.zeros((1, 3))
        self.distance_array = np.zeros(1)
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        #rospy.Subscriber('/traffic_waypoint', )

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.distance_array = np.zeros(self.numpy_list.shape[0])
            for i in range(self.numpy_list.shape[0]):
                self.distance_array[i] = np.linalg.norm(self.current_pose - self.numpy_list[i])
            if (self.numpy_list.shape[0] > 1):
                rospy.loginfo(self.count_waypoint)
                rospy.loginfo(len(self.waypoints.waypoints))
                minimum = np.argmin(self.distance_array)
                minimum = self.transform_and_ahead(self.waypoints.waypoints[minimum], minimum)
                self.pub_wp = Lane()
                for i in range(LOOKAHEAD_WPS):
                    self.pub_wp.waypoints.append(self.waypoints.waypoints[(minimum + i)% len(self.waypoints.waypoints)])
                rospy.loginfo(self.pub_wp.waypoints[0].pose.pose.position)
                rospy.loginfo(self.current_pose)
                self.final_waypoints_pub.publish(self.pub_wp)
            rate.sleep()

        # TODO: Add other member variables you need below

        rospy.spin()

    def pose_cb(self, msg):
        self.current_pose[0] = msg.pose.position.x
        self.current_pose[1] = msg.pose.position.y
        self.current_pose[2] = msg.pose.position.z

        self.current_orientation[0] = msg.pose.orientation.x
        self.current_orientation[1] = msg.pose.orientation.y
        self.current_orientation[2] = msg.pose.orientation.z
        self.current_orientation[3] = msg.pose.orientation.w

        rospy.loginfo('WaypointUpdater: Updated pose - x: %.2f - y: %.2f', msg.pose.position.x, msg.pose.position.y)

        # TODO: Implement

    def waypoints_cb(self, waypoints):
        count_waypoint = len(waypoints.waypoints)
        self.numpy_list = np.zeros((count_waypoint, 3))
        for i in range(len(waypoints.waypoints)):
            self.numpy_list[i][0] = waypoints.waypoints[i].pose.pose.position.x
            self.numpy_list[i][1] = waypoints.waypoints[i].pose.pose.position.y
            self.numpy_list[i][2] = waypoints.waypoints[i].pose.pose.position.z
        self.count_waypoint = count_waypoint
        self.waypoints = waypoints

        # TODO: Implement
        # pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def transform_and_ahead(self, waypoint, id):
        wx = waypoint.pose.pose.position.x
        wy = waypoint.pose.pose.position.y

        rospy.loginfo(self.current_orientation[0])
        rospy.loginfo("---------------------")
        _,_,yaw = tf.transformations.euler_from_quaternion([self.current_orientation[0],self.current_orientation[1],self.current_orientation[2], self.current_orientation[3]])

        dx = wx - self.current_pose[0]
        dy = wy - self.current_pose[1]

        wx_local = math.cos(-yaw)*dx - math.sin(-yaw) * dy

        if wx_local > 0.0:
            return id
        else:
            return id+1

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
